"""
SCARA Control Panel Pro — Contrôle manuel + Asservissement visuel
Fusion de gui_robot.py et main_phase3.py
"""

import customtkinter as ctk
import tkinter as tk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import queue
import threading
import time
import math
import os
import sys
import snap7
from snap7.util import set_real, set_bool, get_bool

# ── Dépendances optionnelles ──────────────────────────────────────────────────
try:
    from PIL import Image, ImageTk
    _PIL = True
except ImportError:
    _PIL = False

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False

# ── Modules Phase2/3 (asservissement visuel) ──────────────────────────────────
_HERE      = os.path.dirname(os.path.abspath(__file__))
_PHASE2    = os.path.join(_HERE, 'python', 'phase2')
_PHASE3    = os.path.join(_HERE, 'python', 'phase3')
_CALIB_DIR = os.path.join(_PHASE2, 'calibration_data')
_VS_AVAIL  = False
_VS_ERR    = ""

if _CV2:
    try:
        for _p in (_PHASE2, _PHASE3):
            if _p not in sys.path:
                sys.path.insert(0, _p)
        from main_phase3 import (
            Phase3Pipeline, PipelineState, STATE_COLORS,
            _load_phase2_assets, CONVERGE_THR_MM, APPROACH_THR_MM,
        )
        _VS_AVAIL = True
    except Exception as _e:
        _VS_ERR = str(_e)

# ── Thème ─────────────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ── Paramètres robot ──────────────────────────────────────────────────────────
L1, L2, Z_AMP = 150, 110, 80
J1_SCALE = 25000 / math.radians(170)
J2_SCALE = 25000 / math.radians(135)
Z_SCALE  = 25000 / Z_AMP
J4_SCALE = 2000  / math.radians(180)

AXIS_COLORS  = ['#e94560', '#ff8c42', '#00d4ff', '#a855f7']
AXIS_NAMES   = ["J1 — Épaule", "J2 — Coude", "Z — Hauteur", "J4 — Orient."]
SPEED_NAMES  = ["J1", "J2", "Z", "J4"]
LIMITS       = [(-25000., 25000.), (-25000., 25000.),
                (-25000., 25000.), (-2000., 2000.)]
INIT_SPEEDS  = [22000., 25000., 22000., 1800.]
SPEED_LIMITS = [(500, 50000), (500, 50000), (500, 50000), (100, 5000)]
BG_3D        = '#0d1117'

def enc_to_physical(enc):
    return (enc[0]/J1_SCALE, enc[1]/J2_SCALE, enc[2]/Z_SCALE, enc[3]/J4_SCALE)


# ── Dialogue de calibration caméra ────────────────────────────────────────────
class CalibrationDialog(ctk.CTkToplevel):
    """Fenêtre de calibration de la caméra par damier."""

    # Drapeaux OpenCV : détection rapide + normalisation + seuillage adaptatif
    _FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH
              | cv2.CALIB_CB_NORMALIZE_IMAGE
              | cv2.CALIB_CB_FAST_CHECK)

    def __init__(self, parent, cam_idx: int):
        super().__init__(parent)
        self.title("Calibration de la caméra")
        self.geometry("960x600")
        self.resizable(True, True)
        self.lift(); self.focus_force()
        self._cam_idx   = cam_idx
        self._cap       = None
        self._running   = False
        self._poll_id   = None
        self._frames    = []
        self._MIN       = 12
        # Taille du damier (coins intérieurs) — modifiable dans l'UI
        self._board_cols = tk.StringVar(value="9")
        self._board_rows = tk.StringVar(value="6")
        self.protocol("WM_DELETE_WINDOW", self._close)
        if not (_CV2 and _PIL):
            ctk.CTkLabel(self, text="OpenCV ou PIL non disponible.",
                         font=ctk.CTkFont("Segoe UI", 13)).pack(expand=True)
            return
        self._build()
        self._start()

    def _build(self):
        self.grid_columnconfigure(0, weight=3)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Aperçu caméra
        pf = ctk.CTkFrame(self, corner_radius=8)
        pf.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=10)
        self._lbl_prev = tk.Label(pf, bg='#0d1117')
        self._lbl_prev.pack(fill="both", expand=True)

        # Panneau de contrôle
        sb = ctk.CTkFrame(self, corner_radius=8, width=210)
        sb.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
        sb.grid_propagate(False)
        sb.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(sb, text="CALIBRATION\nCAMÉRA",
                     font=ctk.CTkFont("Segoe UI", 13, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(18, 4))

        # ── Taille du damier ──────────────────────────────────────────────
        board_row = ctk.CTkFrame(sb, fg_color="transparent")
        board_row.grid(row=1, column=0, padx=10, pady=(4, 0))
        ctk.CTkLabel(board_row, text="Coins :",
                     font=ctk.CTkFont("Segoe UI", 9),
                     text_color="gray").pack(side="left", padx=(0, 4))
        ctk.CTkEntry(board_row, textvariable=self._board_cols, width=38,
                     font=ctk.CTkFont("Consolas", 10)
                     ).pack(side="left")
        ctk.CTkLabel(board_row, text="×",
                     font=ctk.CTkFont("Segoe UI", 10),
                     text_color="gray").pack(side="left", padx=2)
        ctk.CTkEntry(board_row, textvariable=self._board_rows, width=38,
                     font=ctk.CTkFont("Consolas", 10)
                     ).pack(side="left")

        # Préréglages courants
        presets_row = ctk.CTkFrame(sb, fg_color="transparent")
        presets_row.grid(row=2, column=0, padx=10, pady=(2, 0))
        for label, cols, rows in [("9×6", "9", "6"), ("7×6", "7", "6"),
                                   ("10×7", "10", "7"), ("6×4", "6", "4")]:
            ctk.CTkButton(presets_row, text=label, width=44, height=22,
                          fg_color="#1a2535", hover_color="#2a3545",
                          font=ctk.CTkFont("Segoe UI", 8),
                          command=lambda c=cols, r=rows: (
                              self._board_cols.set(c),
                              self._board_rows.set(r))
                          ).pack(side="left", padx=1)

        ctk.CTkLabel(sb,
                     text="Capturez ≥ 12 poses\nvariées (angles, distances).",
                     font=ctk.CTkFont("Segoe UI", 9), text_color="gray",
                     justify="center").grid(row=3, column=0, padx=10, pady=(6, 2))

        self._lbl_count = ctk.CTkLabel(sb, text=f"0 / {self._MIN}",
                                       font=ctk.CTkFont("Consolas", 16, "bold"),
                                       text_color=AXIS_COLORS[2])
        self._lbl_count.grid(row=4, column=0, pady=8)

        self._lbl_det = ctk.CTkLabel(sb, text="Cherche damier...",
                                     font=ctk.CTkFont("Segoe UI", 9),
                                     text_color="gray")
        self._lbl_det.grid(row=5, column=0, pady=2)

        ctk.CTkButton(sb, text="📸  Capturer", height=38,
                      fg_color="#0d2a18", hover_color="#1a4a2a",
                      text_color="#00c851",
                      font=ctk.CTkFont("Segoe UI", 11, "bold"),
                      command=self._capture).grid(row=6, column=0,
                                                  padx=12, pady=4, sticky="ew")
        self._btn_calib = ctk.CTkButton(sb, text="⚙  Calibrer", height=38,
                                        fg_color="#0d1a3a", hover_color="#1a2a5a",
                                        text_color="#00d4ff",
                                        font=ctk.CTkFont("Segoe UI", 11, "bold"),
                                        command=self._run_calibration,
                                        state="disabled")
        self._btn_calib.grid(row=7, column=0, padx=12, pady=4, sticky="ew")
        ctk.CTkButton(sb, text="Fermer", height=32,
                      fg_color="#2a0d0d", hover_color="#4a1a1a",
                      text_color="#ff4444",
                      command=self._close).grid(row=9, column=0,
                                                padx=12, pady=(16, 10), sticky="ew")
        self._lbl_status = ctk.CTkLabel(sb, text="",
                                        font=ctk.CTkFont("Segoe UI", 8),
                                        text_color="gray",
                                        wraplength=180, justify="center")
        self._lbl_status.grid(row=8, column=0, padx=8, pady=2)

    def _start(self):
        self._cap     = cv2.VideoCapture(self._cam_idx)
        self._running = True
        self._poll()

    @property
    def _board(self):
        """Taille du damier (cols, rows) lue depuis les champs UI."""
        try:
            return (int(self._board_cols.get()), int(self._board_rows.get()))
        except ValueError:
            return (9, 6)

    @staticmethod
    def _preprocess(gray):
        """CLAHE pour améliorer la détection en éclairage inégal."""
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(gray)

    def _poll(self):
        if not self._running or self._cap is None:
            return
        ret, frame = self._cap.read()
        if ret:
            board = self._board
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            enhanced = self._preprocess(gray)
            found, corners = cv2.findChessboardCorners(enhanced, board, self._FLAGS)
            disp  = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, board, corners, True)
                cv2.putText(disp, f"Damier {board[0]}x{board[1]} detecte",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                self.after(0, lambda b=board: self._lbl_det.configure(
                    text=f"✓ Damier {b[0]}×{b[1]} détecté",
                    text_color="#00c851"))
            else:
                self.after(0, lambda b=board: self._lbl_det.configure(
                    text=f"Cherche {b[0]}×{b[1]}...", text_color="gray"))
            h, w = disp.shape[:2]
            scale = min(700/w, 500/h)
            small = cv2.resize(disp, (int(w*scale), int(h*scale)))
            img   = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
            photo = ImageTk.PhotoImage(img)
            self._lbl_prev.configure(image=photo)
            self._lbl_prev.image = photo
        self._poll_id = self.after(33, self._poll)

    def _capture(self):
        if self._cap is None:
            return
        ret, frame = self._cap.read()
        if not ret:
            return
        board = self._board
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced = self._preprocess(gray)
        found, _ = cv2.findChessboardCorners(enhanced, board, self._FLAGS)
        if found:
            self._frames.append(frame.copy())
            n = len(self._frames)
            self._lbl_count.configure(text=f"{n} / {self._MIN}")
            self._lbl_status.configure(text=f"Image {n} capturée ✓", text_color="#00c851")
            if n >= self._MIN:
                self._btn_calib.configure(state="normal")
        else:
            self._lbl_status.configure(
                text=f"⚠ Damier {board[0]}×{board[1]} non trouvé",
                text_color="#ffbb33")

    def _run_calibration(self):
        self._btn_calib.configure(state="disabled", text="Calcul en cours...")
        threading.Thread(target=self._do_calibration, daemon=True).start()

    def _do_calibration(self):
        try:
            board = self._board
            cols, rows = board
            objp = np.zeros((cols * rows, 3), np.float32)
            objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
            obj_pts, img_pts = [], []
            crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            for frame in self._frames:
                gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                enhanced = self._preprocess(gray)
                ok, corners = cv2.findChessboardCorners(enhanced, board, self._FLAGS)
                if ok:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1,-1), crit)
                    obj_pts.append(objp)
                    img_pts.append(corners2)
            if len(obj_pts) < 5:
                raise ValueError(f"Seulement {len(obj_pts)} images valides (min 5)")
            h, w = self._frames[0].shape[:2]
            rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
            os.makedirs(_CALIB_DIR, exist_ok=True)
            path = os.path.join(_CALIB_DIR, 'camera_params.npz')
            np.savez(path, K=K, dist=dist, rms=np.float64(rms),
                     img_width=np.int32(w), img_height=np.int32(h))
            self.after(0, lambda r=rms: (
                self._lbl_status.configure(
                    text=f"✓ Calibration OK\nRMS = {r:.3f}\ncamera_params.npz",
                    text_color="#00c851"),
                self._btn_calib.configure(text="✓ Calibré", state="disabled"),
            ))
        except Exception as exc:
            self.after(0, lambda e=str(exc): (
                self._lbl_status.configure(text=f"✗ {e}", text_color="#ff4444"),
                self._btn_calib.configure(text="⚙  Calibrer", state="normal"),
            ))

    def _close(self):
        self._running = False
        if self._poll_id:
            self.after_cancel(self._poll_id)
        if self._cap:
            self._cap.release()
        self.destroy()


# ── Application principale ────────────────────────────────────────────────────
class SCARAControlPanel:

    def __init__(self):
        self.root = ctk.CTk()
        self.root.title("SCARA Control Panel Pro")
        self.root.geometry("1400x900")
        self.root.minsize(1100, 720)

        # PLC
        self.plc         = snap7.client.Client()
        self.ip          = '192.168.0.10'
        self.connected   = False
        self.pince_state = False
        self.send_timer  = None
        self.is_sending  = False

        # Variables manuel
        self.pos_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.pos_disp = [tk.StringVar(value="0.0") for _ in range(4)]
        self.vit_vars = [tk.DoubleVar(value=v) for v in INIT_SPEEDS]
        self.vit_disp = [tk.StringVar(value=f"{v:.0f}") for v in INIT_SPEEDS]
        for i in range(4):
            self.pos_vars[i].trace_add('write',
                lambda *a, idx=i: self.pos_disp[idx].set(
                    f"{self.pos_vars[idx].get():.1f}"))
            self.vit_vars[i].trace_add('write',
                lambda *a, idx=i: self.vit_disp[idx].set(
                    f"{int(self.vit_vars[idx].get())}"))

        # Flags 3D / caméra
        self._3d_pending = False
        self._cam_drag   = None

        # État mode
        self._mode = 'manual'

        # Variables VS
        self._vs_running  = False
        self._vs_thread   = None
        self._frame_q     = queue.Queue(maxsize=3)
        self._cam_poll_id = None
        self._vs_pipeline = None

        # Options VS
        self._vs_cam_idx   = tk.StringVar(value="0")
        self._vs_method    = tk.StringVar(value="aruco")
        self._vs_marker_sz = tk.StringVar(value="0.05")
        self._vs_work_z    = tk.StringVar(value="0.80")
        self._vs_gain      = tk.DoubleVar(value=0.5)
        self._vs_adaptive  = tk.BooleanVar(value=True)
        self._vs_yolo_mdl  = tk.StringVar(value="yolov8n.pt")
        self._vs_ft_x      = tk.StringVar(value="350")
        self._vs_ft_y      = tk.StringVar(value="0")
        self._vs_ft_z      = tk.StringVar(value="-150")

        self._build_ui()
        self._update_3d()

    # ═══════════════════════════════════════════════════════════════════════════
    # UI principale
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self._build_topbar()

        content = ctk.CTkFrame(self.root, fg_color="transparent")
        content.grid(row=1, column=0, sticky="nsew", padx=14, pady=(8, 14))
        content.grid_rowconfigure(0, weight=1)
        content.grid_columnconfigure(0, weight=1)

        # Les deux modes occupent la même cellule
        self._manual_frame = ctk.CTkFrame(content, fg_color="transparent")
        self._manual_frame.grid(row=0, column=0, sticky="nsew")
        self._manual_frame.grid_columnconfigure(0, weight=3)
        self._manual_frame.grid_columnconfigure(1, weight=2)
        self._manual_frame.grid_rowconfigure(0, weight=1)
        self._build_manual_mode(self._manual_frame)

        self._visual_frame = ctk.CTkFrame(content, fg_color="transparent")
        self._visual_frame.grid(row=0, column=0, sticky="nsew")
        self._visual_frame.grid_columnconfigure(0, weight=3)
        self._visual_frame.grid_columnconfigure(1, weight=2)
        self._visual_frame.grid_rowconfigure(0, weight=1)
        self._build_visual_mode(self._visual_frame)
        self._visual_frame.grid_remove()

    def _build_topbar(self):
        bar = ctk.CTkFrame(self.root, height=64, corner_radius=12)
        bar.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 0))
        bar.grid_propagate(False)
        bar.grid_columnconfigure(2, weight=1)

        ctk.CTkLabel(
            bar, text="⬡  SCARA Control Panel Pro",
            font=ctk.CTkFont("Segoe UI", 17, "bold"),
            text_color="#00d4ff"
        ).grid(row=0, column=0, padx=20, sticky="w")

        self._mode_btn = ctk.CTkSegmentedButton(
            bar, values=["  MANUEL  ", "  VISUEL  "],
            command=self._switch_mode,
            font=ctk.CTkFont("Segoe UI", 11, "bold"),
            width=240, height=34,
            selected_color="#1a3a5a", selected_hover_color="#2a4a7a",
            unselected_color="#1a1a2a", unselected_hover_color="#252535",
        )
        self._mode_btn.set("  MANUEL  ")
        self._mode_btn.grid(row=0, column=1, padx=20, sticky="w")

        ctk.CTkLabel(bar, text=f"PLC  {self.ip}",
                     font=ctk.CTkFont("Consolas", 9),
                     text_color="gray").grid(row=0, column=2, sticky="w", padx=8)

        right = ctk.CTkFrame(bar, fg_color="transparent")
        right.grid(row=0, column=3, padx=14, sticky="e")

        self.lbl_status = ctk.CTkLabel(right, text="●  DÉCONNECTÉ",
                                       font=ctk.CTkFont("Segoe UI", 11, "bold"),
                                       text_color="#ff4444")
        self.lbl_status.pack(side="left", padx=(0, 18))

        ctk.CTkButton(right, text="Connecter", width=105, height=34,
                      fg_color="#0d2a18", hover_color="#1a4a2a",
                      text_color="#00c851",
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      corner_radius=8,
                      command=self.connect_plc).pack(side="left", padx=3)

        ctk.CTkButton(right, text="Déconnecter", width=105, height=34,
                      fg_color="#2a0d0d", hover_color="#4a1a1a",
                      text_color="#ff4444",
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      corner_radius=8,
                      command=self.disconnect_plc).pack(side="left", padx=3)

    def _switch_mode(self, label: str):
        mode = 'visual' if 'VISUEL' in label else 'manual'
        if mode == self._mode:
            return
        self._mode = mode
        if mode == 'visual':
            self._manual_frame.grid_remove()
            self._visual_frame.grid()
        else:
            if self._vs_running:
                self._stop_vs()
            self._visual_frame.grid_remove()
            self._manual_frame.grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # MODE MANUEL : vue 3D + curseurs
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_manual_mode(self, parent):
        self._build_3d_panel(parent)
        self._build_controls_panel(parent)

    def _build_3d_panel(self, parent):
        panel = ctk.CTkFrame(parent, corner_radius=12)
        panel.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        panel.grid_rowconfigure(1, weight=1)
        panel.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(panel, text="VUE 3D — ROBOT SCARA",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(12, 0))

        self.fig = plt.Figure(figsize=(6.4, 5.6), dpi=96)
        self.fig.patch.set_facecolor(BG_3D)
        self.ax3d = self.fig.add_subplot(111, projection='3d')
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

        cw = ctk.CTkFrame(panel, fg_color="transparent")
        cw.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)

        self.fig_canvas = FigureCanvasTkAgg(self.fig, master=cw)
        self.fig_canvas.get_tk_widget().configure(bg=BG_3D, highlightthickness=0)
        self.fig_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._setup_cam_ctrl()

        self._build_coord_bar(panel, row=2)

    def _build_coord_bar(self, parent, row):
        bar = ctk.CTkFrame(parent, height=64, corner_radius=10)
        bar.grid(row=row, column=0, sticky="ew", padx=10, pady=(0, 10))
        bar.grid_propagate(False)
        bar.grid_columnconfigure((0, 1, 2, 3), weight=1)
        self.coord_labels = []
        for j, (name, col) in enumerate([
            ("X (mm)", AXIS_COLORS[0]), ("Y (mm)", AXIS_COLORS[1]),
            ("Z (mm)", AXIS_COLORS[2]), ("θ (°)",  AXIS_COLORS[3]),
        ]):
            f = ctk.CTkFrame(bar, corner_radius=7)
            f.grid(row=0, column=j, padx=5, pady=8, sticky="nsew")
            ctk.CTkLabel(f, text=name, font=ctk.CTkFont("Segoe UI", 8),
                         text_color="gray").pack(pady=(4, 0))
            lbl = ctk.CTkLabel(f, text="0.0",
                               font=ctk.CTkFont("Consolas", 13, "bold"),
                               text_color=col)
            lbl.pack(pady=(0, 4))
            self.coord_labels.append(lbl)

    # Contrôle caméra 3D (tkinter natif, sans basculement)
    def _setup_cam_ctrl(self):
        try:
            for cid in self.ax3d._cids:
                self.fig_canvas.mpl_disconnect(cid)
            self.ax3d._cids = []
        except Exception:
            pass
        self.ax3d.view_init(elev=25, azim=-55)
        w = self.fig_canvas.get_tk_widget()
        w.bind('<ButtonPress-1>',   self._cam_press_tk)
        w.bind('<B1-Motion>',       self._cam_move_tk)
        w.bind('<ButtonRelease-1>', self._cam_release_tk)

    def _cam_press_tk(self, event):
        self._cam_drag = (event.x, event.y, self.ax3d.azim, self.ax3d.elev)
        return 'break'

    def _cam_move_tk(self, event):
        if self._cam_drag is None:
            return 'break'
        x0, y0, a0, e0 = self._cam_drag
        self.ax3d.view_init(
            elev=max(5, min(80, e0 + (event.y - y0) * 0.4)),
            azim=a0 - (event.x - x0) * 0.5)
        self.fig_canvas.draw_idle()
        return 'break'

    def _cam_release_tk(self, event):
        self._cam_drag = None
        return 'break'

    def _build_controls_panel(self, parent):
        panel = ctk.CTkFrame(parent, corner_radius=12)
        panel.grid(row=0, column=1, sticky="nsew", padx=(7, 0))
        panel.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(panel, text="CONTRÔLES",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(12, 6))

        for i in range(4):
            self._build_axis_card(panel, i, row=i + 1)

        pw = ctk.CTkFrame(panel, fg_color="transparent")
        pw.grid(row=5, column=0, sticky="ew", padx=16, pady=(14, 4))
        pw.grid_columnconfigure(0, weight=1)
        self.btn_pince = ctk.CTkButton(
            pw, text="⊙  PINCE : OUVERTE", height=52,
            fg_color="#2e2410", hover_color="#4a3a18", text_color="#ffbb33",
            font=ctk.CTkFont("Segoe UI", 14, "bold"),
            corner_radius=10, command=self.toggle_pince)
        self.btn_pince.grid(row=0, column=0, sticky="ew")

        self._build_speed_panel(panel, row=6)

        self.lbl_send = ctk.CTkLabel(panel, text="",
                                     font=ctk.CTkFont("Segoe UI", 9),
                                     text_color="#00d4ff")
        self.lbl_send.grid(row=7, column=0, pady=(4, 10))

    def _build_axis_card(self, parent, i, row):
        card = ctk.CTkFrame(parent, corner_radius=10)
        card.grid(row=row, column=0, sticky="ew", padx=16, pady=4)
        card.grid_columnconfigure(0, weight=1)

        top = ctk.CTkFrame(card, fg_color="transparent")
        top.grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 2))
        top.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(top, text="●", font=ctk.CTkFont("Segoe UI", 14),
                     text_color=AXIS_COLORS[i]).grid(row=0, column=0, padx=(0, 6))
        ctk.CTkLabel(top, text=AXIS_NAMES[i],
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color=AXIS_COLORS[i]).grid(row=0, column=1, sticky="w")
        ctk.CTkLabel(top, textvariable=self.pos_disp[i],
                     font=ctk.CTkFont("Consolas", 12, "bold"),
                     text_color="white", width=95,
                     anchor="e").grid(row=0, column=2, sticky="e")

        sl = ctk.CTkFrame(card, fg_color="transparent")
        sl.grid(row=1, column=0, sticky="ew", padx=14, pady=(2, 10))
        sl.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(sl, text=str(int(LIMITS[i][0])),
                     font=ctk.CTkFont("Consolas", 8),
                     text_color="#556070").grid(row=0, column=0)
        ctk.CTkSlider(sl, from_=LIMITS[i][0], to=LIMITS[i][1],
                      variable=self.pos_vars[i], command=self.on_slider_change,
                      button_color=AXIS_COLORS[i],
                      button_hover_color=AXIS_COLORS[i],
                      progress_color=AXIS_COLORS[i],
                      height=18).grid(row=0, column=1, sticky="ew", padx=8)
        ctk.CTkLabel(sl, text=str(int(LIMITS[i][1])),
                     font=ctk.CTkFont("Consolas", 8),
                     text_color="#556070").grid(row=0, column=2)

    def _build_speed_panel(self, parent, row):
        frame = ctk.CTkFrame(parent, fg_color="transparent")
        frame.grid(row=row, column=0, sticky="ew", padx=16, pady=(8, 0))
        frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(frame, text="VITESSES (enc/s)",
                     font=ctk.CTkFont("Segoe UI", 9, "bold"),
                     text_color="gray").grid(row=0, column=0, sticky="w", pady=(0, 3))
        for i, name in enumerate(SPEED_NAMES):
            card = ctk.CTkFrame(frame, corner_radius=8)
            card.grid(row=i + 1, column=0, sticky="ew", pady=2)
            card.grid_columnconfigure(0, weight=1)
            top = ctk.CTkFrame(card, fg_color="transparent")
            top.grid(row=0, column=0, sticky="ew", padx=10, pady=(5, 1))
            top.grid_columnconfigure(1, weight=1)
            ctk.CTkLabel(top, text=f"● {name}",
                         font=ctk.CTkFont("Segoe UI", 9, "bold"),
                         text_color=AXIS_COLORS[i]).grid(row=0, column=0, padx=(0, 6))
            ctk.CTkLabel(top, textvariable=self.vit_disp[i],
                         font=ctk.CTkFont("Consolas", 10, "bold"),
                         text_color="white", width=65,
                         anchor="e").grid(row=0, column=2, sticky="e")
            sl = ctk.CTkFrame(card, fg_color="transparent")
            sl.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
            sl.grid_columnconfigure(1, weight=1)
            ctk.CTkLabel(sl, text=str(SPEED_LIMITS[i][0]),
                         font=ctk.CTkFont("Consolas", 7),
                         text_color="#556070").grid(row=0, column=0)
            ctk.CTkSlider(sl, from_=SPEED_LIMITS[i][0], to=SPEED_LIMITS[i][1],
                          variable=self.vit_vars[i],
                          button_color=AXIS_COLORS[i],
                          button_hover_color=AXIS_COLORS[i],
                          progress_color=AXIS_COLORS[i],
                          height=14).grid(row=0, column=1, sticky="ew", padx=6)
            ctk.CTkLabel(sl, text=str(SPEED_LIMITS[i][1]),
                         font=ctk.CTkFont("Consolas", 7),
                         text_color="#556070").grid(row=0, column=2)

    # ═══════════════════════════════════════════════════════════════════════════
    # MODE VISUEL : flux caméra + options
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_visual_mode(self, parent):
        # ── Panneau gauche : flux caméra ──────────────────────────────────────
        left = ctk.CTkFrame(parent, corner_radius=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        left.grid_rowconfigure(1, weight=1)
        left.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(left, text="FLUX CAMÉRA — ASSERVISSEMENT VISUEL",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(12, 0))

        cam_wrap = ctk.CTkFrame(left, fg_color="transparent")
        cam_wrap.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)
        self._cam_display = tk.Label(cam_wrap, bg=BG_3D,
                                     text="Caméra non démarrée",
                                     fg="#3a5070",
                                     font=("Segoe UI", 14))
        self._cam_display.pack(fill="both", expand=True)

        self._build_vs_state_bar(left, row=2)

        # ── Panneau droit : options ───────────────────────────────────────────
        right = ctk.CTkFrame(parent, corner_radius=12)
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 0))
        right.grid_columnconfigure(0, weight=1)
        self._build_vs_options(right)

    def _build_vs_state_bar(self, parent, row):
        bar = ctk.CTkFrame(parent, height=64, corner_radius=10)
        bar.grid(row=row, column=0, sticky="ew", padx=10, pady=(0, 10))
        bar.grid_propagate(False)
        bar.grid_columnconfigure((0, 1, 2, 3), weight=1)
        self._vs_bar_labels = []
        for j, (name, col) in enumerate([
            ("État", "#00d4ff"), ("Erreur t (mm)", "#ff8c42"),
            ("EE  X, Y (mm)", AXIS_COLORS[0]), ("EE  Z (mm)", AXIS_COLORS[2]),
        ]):
            f = ctk.CTkFrame(bar, corner_radius=7)
            f.grid(row=0, column=j, padx=5, pady=8, sticky="nsew")
            ctk.CTkLabel(f, text=name, font=ctk.CTkFont("Segoe UI", 8),
                         text_color="gray").pack(pady=(4, 0))
            lbl = ctk.CTkLabel(f, text="—",
                               font=ctk.CTkFont("Consolas", 11, "bold"),
                               text_color=col)
            lbl.pack(pady=(0, 4))
            self._vs_bar_labels.append(lbl)

    def _build_vs_options(self, parent):
        parent.grid_rowconfigure(10, weight=1)

        ctk.CTkLabel(parent, text="ASSERVISSEMENT VISUEL",
                     font=ctk.CTkFont("Segoe UI", 11, "bold"),
                     text_color="#00d4ff"
                     ).grid(row=0, column=0, pady=(12, 4), padx=16, sticky="w")

        # ── Source caméra ────────────────────────────────────────────────────
        self._sep(parent, "CAMÉRA", row=1)
        src = ctk.CTkFrame(parent, fg_color="transparent")
        src.grid(row=2, column=0, sticky="ew", padx=16, pady=3)
        src.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(src, text="Index caméra :",
                     font=ctk.CTkFont("Segoe UI", 10),
                     text_color="gray").grid(row=0, column=0, padx=(0, 8))
        ctk.CTkEntry(src, textvariable=self._vs_cam_idx, width=55,
                     font=ctk.CTkFont("Consolas", 11)
                     ).grid(row=0, column=1, sticky="w")
        ctk.CTkButton(src, text="🎯 Calibrer caméra", width=140, height=28,
                      fg_color="#0d1a3a", hover_color="#1a2a5a",
                      text_color="#00d4ff",
                      font=ctk.CTkFont("Segoe UI", 9, "bold"),
                      command=self._open_calibration
                      ).grid(row=0, column=2, padx=(10, 0))

        # ── Méthode de détection ─────────────────────────────────────────────
        self._sep(parent, "MÉTHODE DE DÉTECTION", row=3)
        meth = ctk.CTkFrame(parent, fg_color="transparent")
        meth.grid(row=4, column=0, sticky="ew", padx=16, pady=3)
        for j, (val, lbl) in enumerate([
            ("aruco",        "ArUco"),
            ("yolo",         "YOLO"),
            ("force_target", "Cible forcée"),
        ]):
            ctk.CTkRadioButton(meth, text=lbl, variable=self._vs_method, value=val,
                               font=ctk.CTkFont("Segoe UI", 10),
                               command=self._on_method_change,
                               radiobutton_width=16,
                               radiobutton_height=16).grid(row=0, column=j, padx=8)

        # Sous-options conditionnelles
        self._meth_sub = ctk.CTkFrame(parent, fg_color="transparent")
        self._meth_sub.grid(row=5, column=0, sticky="ew", padx=16, pady=2)
        self._meth_sub.grid_columnconfigure(1, weight=1)
        self._on_method_change()

        # ── Paramètres ───────────────────────────────────────────────────────
        self._sep(parent, "PARAMÈTRES", row=6)
        params = ctk.CTkFrame(parent, fg_color="transparent")
        params.grid(row=7, column=0, sticky="ew", padx=16, pady=3)
        params.grid_columnconfigure(1, weight=1)
        for r, (lbl, var) in enumerate([
            ("Taille marqueur (m) :", self._vs_marker_sz),
            ("Plan de travail Z (m) :", self._vs_work_z),
        ]):
            ctk.CTkLabel(params, text=lbl, font=ctk.CTkFont("Segoe UI", 9),
                         text_color="gray").grid(row=r, column=0, sticky="w", pady=2)
            ctk.CTkEntry(params, textvariable=var, width=75,
                         font=ctk.CTkFont("Consolas", 10)
                         ).grid(row=r, column=1, padx=(8, 0), sticky="w", pady=2)

        gain_row = ctk.CTkFrame(parent, fg_color="transparent")
        gain_row.grid(row=8, column=0, sticky="ew", padx=16, pady=(6, 2))
        gain_row.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(gain_row, text="Gain :",
                     font=ctk.CTkFont("Segoe UI", 9),
                     text_color="gray").grid(row=0, column=0, padx=(0, 8))
        ctk.CTkSlider(gain_row, from_=0.05, to=2.0, variable=self._vs_gain,
                      button_color="#00d4ff", progress_color="#00d4ff",
                      height=14).grid(row=0, column=1, sticky="ew")
        self._lbl_gain = ctk.CTkLabel(gain_row, text="0.50",
                                      font=ctk.CTkFont("Consolas", 10),
                                      text_color="#00d4ff", width=42)
        self._lbl_gain.grid(row=0, column=2, padx=(6, 0))
        self._vs_gain.trace_add('write', lambda *_: self._lbl_gain.configure(
            text=f"{self._vs_gain.get():.2f}"))

        ctk.CTkCheckBox(parent, text="Gain adaptatif",
                        variable=self._vs_adaptive,
                        font=ctk.CTkFont("Segoe UI", 10),
                        text_color="gray").grid(row=9, column=0, padx=16,
                                                pady=2, sticky="w")

        # ── Bouton START / STOP ──────────────────────────────────────────────
        btn_wrap = ctk.CTkFrame(parent, fg_color="transparent")
        btn_wrap.grid(row=11, column=0, sticky="ew", padx=16, pady=(14, 4))
        btn_wrap.grid_columnconfigure(0, weight=1)

        if not _VS_AVAIL:
            ctk.CTkLabel(btn_wrap,
                         text=f"⚠ {'OpenCV manquant' if not _CV2 else _VS_ERR[:60]}",
                         font=ctk.CTkFont("Segoe UI", 9), text_color="#ff8c42",
                         wraplength=260).grid(row=0, column=0, pady=4)

        self._btn_vs = ctk.CTkButton(
            btn_wrap, text="▶  DÉMARRER L'ASSERVISSEMENT", height=52,
            fg_color="#0d2a18", hover_color="#1a4a2a", text_color="#00c851",
            font=ctk.CTkFont("Segoe UI", 13, "bold"),
            corner_radius=10, command=self._toggle_vs,
            state="normal" if _VS_AVAIL else "disabled")
        self._btn_vs.grid(row=1, column=0, sticky="ew")

        self._lbl_vs_status = ctk.CTkLabel(parent, text="",
                                            font=ctk.CTkFont("Segoe UI", 9),
                                            text_color="#00d4ff")
        self._lbl_vs_status.grid(row=12, column=0, pady=(4, 10))

    def _sep(self, parent, title, row):
        f = ctk.CTkFrame(parent, height=22, fg_color="transparent")
        f.grid(row=row, column=0, sticky="ew", padx=16, pady=(8, 0))
        f.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(f, text=title,
                     font=ctk.CTkFont("Segoe UI", 8, "bold"),
                     text_color="#3a5070").grid(row=0, column=0, padx=(0, 8))
        ctk.CTkFrame(f, height=1, fg_color="#1a2535").grid(row=0, column=1, sticky="ew")

    def _on_method_change(self, *_):
        for w in self._meth_sub.winfo_children():
            w.destroy()
        self._meth_sub.grid_columnconfigure(1, weight=1)
        method = self._vs_method.get()

        if method == "yolo":
            ctk.CTkLabel(self._meth_sub, text="Modèle YOLO :",
                         font=ctk.CTkFont("Segoe UI", 9),
                         text_color="gray").grid(row=0, column=0, padx=(0, 6))
            ctk.CTkEntry(self._meth_sub, textvariable=self._vs_yolo_mdl, width=170,
                         font=ctk.CTkFont("Consolas", 10)
                         ).grid(row=0, column=1, sticky="w")

        elif method == "force_target":
            ctk.CTkLabel(self._meth_sub, text="Cible en mm  ( X / Y / Z ) :",
                         font=ctk.CTkFont("Segoe UI", 9),
                         text_color="gray").grid(row=0, column=0, columnspan=7,
                                                 sticky="w", pady=(0, 4))
            for j, (lbl, var) in enumerate([
                ("X", self._vs_ft_x), ("Y", self._vs_ft_y), ("Z", self._vs_ft_z)
            ]):
                ctk.CTkLabel(self._meth_sub, text=lbl,
                             font=ctk.CTkFont("Consolas", 10),
                             text_color=AXIS_COLORS[j]
                             ).grid(row=1, column=j*2, padx=(0, 2))
                ctk.CTkEntry(self._meth_sub, textvariable=var, width=70,
                             font=ctk.CTkFont("Consolas", 10)
                             ).grid(row=1, column=j*2+1, padx=(0, 10))

    def _open_calibration(self):
        try:
            cam_idx = int(self._vs_cam_idx.get())
        except ValueError:
            cam_idx = 0
        dlg = CalibrationDialog(self.root, cam_idx=cam_idx)
        dlg.grab_set()

    # ═══════════════════════════════════════════════════════════════════════════
    # THREAD ASSERVISSEMENT VISUEL
    # ═══════════════════════════════════════════════════════════════════════════

    def _toggle_vs(self):
        if self._vs_running:
            self._stop_vs()
        else:
            self._start_vs()

    def _start_vs(self):
        self._vs_running = True
        self._btn_vs.configure(text="■  ARRÊTER",
                               fg_color="#2a0d0d", hover_color="#4a1a1a",
                               text_color="#ff4444")
        self._lbl_vs_status.configure(text="Initialisation...",
                                      text_color="#ffbb33")
        while not self._frame_q.empty():
            try: self._frame_q.get_nowait()
            except Exception: break
        self._vs_thread = threading.Thread(target=self._vs_worker, daemon=True)
        self._vs_thread.start()
        self._cam_poll_id = self.root.after(33, self._poll_vs_frame)

    def _stop_vs(self, error: str = ""):
        self._vs_running = False
        if self._cam_poll_id:
            self.root.after_cancel(self._cam_poll_id)
            self._cam_poll_id = None
        self._btn_vs.configure(text="▶  DÉMARRER L'ASSERVISSEMENT",
                               fg_color="#0d2a18", hover_color="#1a4a2a",
                               text_color="#00c851")
        if error:
            self._lbl_vs_status.configure(text=f"✗  {error[:120]}",
                                           text_color="#ff4444")
        elif self._lbl_vs_status.cget("text").startswith("✗"):
            pass  # conserver le message d'erreur
        else:
            self._lbl_vs_status.configure(text="Arrêté", text_color="gray")
        self._cam_display.configure(image="", text="Caméra arrêtée", fg="#3a5070")

    def _vs_worker(self):
        cap = None
        try:
            cam_idx = int(self._vs_cam_idx.get())
            method  = self._vs_method.get()
            ft      = None
            actual_method = method

            if method == "force_target":
                actual_method = "aruco"
                ft = np.array([float(self._vs_ft_x.get()),
                               float(self._vs_ft_y.get()),
                               float(self._vs_ft_z.get())])

            calib_path = os.path.join(_CALIB_DIR, 'camera_params.npz')
            tf_path    = os.path.join(_CALIB_DIR, 'cam_to_robot.npz')
            cam_params, robot_tf = _load_phase2_assets(calib_path, tf_path)

            pipeline = Phase3Pipeline(
                cam_params   = cam_params,
                robot_tf     = robot_tf,
                method       = actual_method,
                marker_size  = float(self._vs_marker_sz.get()),
                work_plane_z = float(self._vs_work_z.get()),
                yolo_model   = self._vs_yolo_mdl.get() if actual_method == "yolo" else None,
                force_target = ft,
                dt           = 1.0 / 30,
            )
            pipeline.ctrl.tune(gain=self._vs_gain.get())
            self._vs_pipeline = pipeline

            cap = cv2.VideoCapture(cam_idx)
            if not cap.isOpened():
                self.root.after(0, lambda: self._lbl_vs_status.configure(
                    text=f"✗ Caméra {cam_idx} introuvable", text_color="#ff4444"))
                return

            self.root.after(0, lambda: self._lbl_vs_status.configure(
                text="● En cours", text_color="#00c851"))

            while self._vs_running:
                ret, frame = cap.read()
                if not ret:
                    break

                annotated, cmd, state = pipeline.process_frame(frame)

                # ── Envoi PLC ──────────────────────────────────────────────
                if self.connected and cmd is not None:
                    try:
                        q = pipeline.q_current
                        data = bytearray(32)
                        set_real(data,  0, float(math.degrees(q[0])))
                        set_real(data,  4, float(q[1] * 1000.0))
                        set_real(data,  8, float(math.degrees(q[2])))
                        set_real(data, 12, 45.0)
                        set_real(data, 16, float(self.vit_vars[0].get()))
                        set_real(data, 20, float(self.vit_vars[1].get()))
                        set_real(data, 24, float(self.vit_vars[2].get()))
                        set_real(data, 28, float(self.vit_vars[3].get()))
                        self.plc.write_area(0x83, 0, 100, data)

                        gripper_close = (
                            pipeline.sequencer.gripper.state.name
                            in ("CLOSED", "CLOSING"))
                        m34 = self.plc.read_area(0x83, 0, 34, 1)
                        set_bool(m34, 0, 5, gripper_close)
                        self.plc.write_area(0x83, 0, 34, m34)

                        if state not in (PipelineState.SEARCHING,
                                         PipelineState.EMERGENCY):
                            m50 = bytearray(1)
                            set_bool(m50, 0, 0, True)
                            self.plc.write_area(0x83, 0, 50, m50)
                            time.sleep(0.01)
                            set_bool(m50, 0, 0, False)
                            self.plc.write_area(0x83, 0, 50, m50)
                    except Exception:
                        pass

                # ── Mise à jour barre d état ───────────────────────────────
                try:
                    ee   = pipeline._dbg_t_cur * 1000
                    bgr  = STATE_COLORS.get(state, (100, 100, 100))
                    hcol = f"#{bgr[2]:02x}{bgr[1]:02x}{bgr[0]:02x}"
                    e_mm = f"{pipeline.errors[-1].norm_t_mm:.1f}" if pipeline.errors else "—"
                    s, xy, z_s = (
                        state.name,
                        f"{ee[0]:+.0f}, {ee[1]:+.0f}",
                        f"{ee[2]:+.0f}",
                    )
                    self.root.after(0, lambda s=s, e=e_mm, xy=xy, z=z_s, c=hcol: (
                        self._vs_bar_labels[0].configure(text=s, text_color=c),
                        self._vs_bar_labels[1].configure(text=e),
                        self._vs_bar_labels[2].configure(text=xy),
                        self._vs_bar_labels[3].configure(text=z),
                    ))
                except Exception:
                    pass

                # ── File d image ───────────────────────────────────────────
                try:
                    self._frame_q.put_nowait(annotated)
                except queue.Full:
                    try:
                        self._frame_q.get_nowait()
                        self._frame_q.put_nowait(annotated)
                    except Exception:
                        pass

        except Exception as exc:
            import traceback
            err_msg = str(exc) or traceback.format_exc().splitlines()[-1]
            self.root.after(0, lambda e=err_msg: self._stop_vs(error=e))
        else:
            self.root.after(0, self._stop_vs)
        finally:
            if cap is not None:
                cap.release()
            self._vs_pipeline = None
            self._vs_running   = False

    def _poll_vs_frame(self):
        if self._mode != 'visual' or not (_CV2 and _PIL):
            return
        try:
            frame = self._frame_q.get_nowait()
            tw = self._cam_display.winfo_width()
            th = self._cam_display.winfo_height()
            if tw > 10 and th > 10:
                h, w  = frame.shape[:2]
                scale = min(tw / w, th / h)
                frame = cv2.resize(frame, (max(1, int(w*scale)),
                                           max(1, int(h*scale))))
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(Image.fromarray(rgb))
            self._cam_display.configure(image=photo, text="")
            self._cam_display.image = photo
        except queue.Empty:
            pass
        except Exception:
            pass
        if self._vs_running:
            self._cam_poll_id = self.root.after(33, self._poll_vs_frame)

    # ═══════════════════════════════════════════════════════════════════════════
    # PLC (logique inchangée)
    # ═══════════════════════════════════════════════════════════════════════════

    def connect_plc(self):
        threading.Thread(target=self._do_connect, daemon=True).start()

    def _do_connect(self):
        self.root.after(0, lambda: self.lbl_status.configure(
            text="●  CONNEXION...", text_color="#ffbb33"))
        try:
            self.plc.connect(self.ip, 0, 1)
            self.connected = True
            self.root.after(0, lambda: self.lbl_status.configure(
                text="●  CONNECTÉ", text_color="#00c851"))
            self.root.after(0, self.read_pince)
        except Exception:
            self.root.after(0, lambda: self.lbl_status.configure(
                text="●  ERREUR", text_color="#ff4444"))

    def disconnect_plc(self):
        if self.connected:
            self.plc.disconnect()
            self.connected = False
        self.lbl_status.configure(text="●  DÉCONNECTÉ", text_color="#ff4444")

    def read_pince(self):
        try:
            m34 = self.plc.read_area(0x83, 0, 34, 1)
            self.pince_state = get_bool(m34, 0, 5)
            self._update_pince_btn()
        except Exception:
            pass

    def _update_pince_btn(self):
        if self.pince_state:
            self.btn_pince.configure(text="⊗  PINCE : FERMÉE",
                                     fg_color="#2a1200", text_color="#ff8800")
        else:
            self.btn_pince.configure(text="⊙  PINCE : OUVERTE",
                                     fg_color="#2e2410", text_color="#ffbb33")

    def toggle_pince(self):
        if not self.connected:
            return
        try:
            self.pince_state = not self.pince_state
            m34 = self.plc.read_area(0x83, 0, 34, 1)
            set_bool(m34, 0, 5, self.pince_state)
            self.plc.write_area(0x83, 0, 34, m34)
            self._update_pince_btn()
            self._update_3d()
        except Exception:
            pass

    def on_slider_change(self, _=None):
        if not self._3d_pending:
            self._3d_pending = True
            self.root.after(50, self._scheduled_3d_update)
        if not self.connected:
            return
        if self.send_timer is not None:
            self.root.after_cancel(self.send_timer)
        self.send_timer = self.root.after(200, self._trigger_move)

    def _scheduled_3d_update(self):
        self._3d_pending = False
        self._update_3d()

    def _trigger_move(self):
        if not self.is_sending:
            threading.Thread(target=self._send_to_plc, daemon=True).start()

    def _send_to_plc(self):
        self.is_sending = True
        self.root.after(0, lambda: self.lbl_send.configure(text="↑  Envoi en cours..."))
        try:
            data = bytearray(32)
            for i in range(4):
                set_real(data, i*4,    float(self.pos_vars[i].get()))
                set_real(data, 16+i*4, float(self.vit_vars[i].get()))
            self.plc.write_area(0x83, 0, 100, data)
            m50 = self.plc.read_area(0x83, 0, 50, 1)
            set_bool(m50, 0, 0, True)
            self.plc.write_area(0x83, 0, 50, m50)
            time.sleep(0.3)
            set_bool(m50, 0, 0, False)
            self.plc.write_area(0x83, 0, 50, m50)
            self.root.after(0, lambda: self.lbl_send.configure(
                text="✓  Commande envoyée"))
        except Exception:
            self.root.after(0, lambda: self.lbl_send.configure(
                text="✗  Erreur d envoi"))
        finally:
            self.is_sending = False

    # ═══════════════════════════════════════════════════════════════════════════
    # VUE 3D (inchangée)
    # ═══════════════════════════════════════════════════════════════════════════

    def _update_3d(self):
        enc = [v.get() for v in self.pos_vars]
        t1, t2, z_vis, t4 = enc_to_physical(enc)
        azim, elev = self.ax3d.azim, self.ax3d.elev
        ax = self.ax3d
        ax.cla()

        ax.set_facecolor(BG_3D)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor('#1a2535')
        ax.tick_params(colors='#3a5070', labelsize=7)
        for obj in (ax.xaxis, ax.yaxis, ax.zaxis):
            obj.label.set_color('#3a5070')
        ax.grid(True, color='#131c2b', linewidth=0.5, linestyle='--')

        BASE_H, COL_H = 15, 80
        ARM_Z = BASE_H + COL_H + z_vis

        self._cyl(ax, 0, 0, 0, BASE_H, 30, '#3a4556', 0.90)
        self._cyl(ax, 0, 0, BASE_H, COL_H + z_vis + Z_AMP, 9,
                  AXIS_COLORS[2], 0.45)
        self._ring(ax, 0, 0, ARM_Z, 18, AXIS_COLORS[0])

        x1 = L1 * math.cos(t1);  y1 = L1 * math.sin(t1)
        ax.plot([0, x1], [0, y1], [0, 0],
                color='#1a2535', linewidth=3, linestyle='--', alpha=0.5)
        ax.plot([0, x1], [0, y1], [ARM_Z]*2,
                color=AXIS_COLORS[0], linewidth=10, solid_capstyle='round',
                zorder=4, alpha=0.95)
        self._ring(ax, x1, y1, ARM_Z, 14, AXIS_COLORS[1])

        x2 = x1 + L2*math.cos(t1+t2);  y2 = y1 + L2*math.sin(t1+t2)
        ax.plot([x1, x2], [y1, y2], [0, 0],
                color='#1a2535', linewidth=2, linestyle='--', alpha=0.4)
        ax.plot([x1, x2], [y1, y2], [ARM_Z]*2,
                color=AXIS_COLORS[1], linewidth=7, solid_capstyle='round',
                zorder=4, alpha=0.95)
        ax.scatter([x2], [y2], [ARM_Z],
                   color='#cbd5e1', s=200, zorder=7, marker='o',
                   depthshade=False, edgecolors='white', linewidths=1.2)

        wrist = t1 + t2 + t4
        self._draw_gripper(ax, x2, y2, ARM_Z, wrist)

        ang = np.linspace(0, 2*math.pi, 180)
        for r, a in ((L1+L2, 0.5), (abs(L1-L2), 0.3)):
            ax.plot(r*np.cos(ang), r*np.sin(ang), np.full(180, ARM_Z),
                    color='#1e3a5f', linewidth=0.8, linestyle=':', alpha=a)
        ax.plot([0,0], [0,0], [0, ARM_Z+10],
                color='#2a4060', linewidth=0.8, linestyle=':', alpha=0.6)

        ax.set_xlabel("X (mm)", fontsize=8, labelpad=4)
        ax.set_ylabel("Y (mm)", fontsize=8, labelpad=4)
        ax.set_zlabel("Z (mm)", fontsize=8, labelpad=4)
        ax.set_xlim(-320, 320);  ax.set_ylim(-320, 320);  ax.set_zlim(-10, 230)
        ax.set_title("Robot SCARA", color="#00d4ff", fontsize=10,
                     pad=6, fontweight='bold')

        self.coord_labels[0].configure(text=f"{x2:.1f}")
        self.coord_labels[1].configure(text=f"{y2:.1f}")
        self.coord_labels[2].configure(text=f"{z_vis:.1f}")
        self.coord_labels[3].configure(text=f"{math.degrees(wrist):.1f}")
        ax.view_init(elev=elev, azim=azim)
        self.fig_canvas.draw_idle()

    def _cyl(self, ax, cx, cy, z_bot, h, r, color, alpha=1.0, n=32):
        theta = np.linspace(0, 2*math.pi, n)
        xc = cx + r*np.cos(theta);  yc = cy + r*np.sin(theta)
        ax.plot_surface(
            np.vstack([xc, xc]), np.vstack([yc, yc]),
            np.vstack([np.full(n, z_bot), np.full(n, z_bot+h)]),
            color=color, alpha=alpha, linewidth=0, antialiased=True)
        r_arr = np.array([0.0, float(r)])
        T, R  = np.meshgrid(theta, r_arr)
        ax.plot_surface(cx+R*np.cos(T), cy+R*np.sin(T),
                        np.full_like(T, z_bot+h),
                        color=color, alpha=alpha*0.75, linewidth=0, antialiased=True)

    def _ring(self, ax, cx, cy, z, r, color):
        theta = np.linspace(0, 2*math.pi, 60)
        ax.plot(cx+r*np.cos(theta), cy+r*np.sin(theta), np.full(60, z),
                color=color, linewidth=2, alpha=0.8, zorder=5)

    def _draw_gripper(self, ax, cx, cy, z, angle):
        gap       = 18 if not self.pince_state else 5
        flen      = 38
        cg        = '#ff4444' if self.pince_state else AXIS_COLORS[3]
        perp      = angle + math.pi / 2
        wx0, wy0  = cx - gap*math.cos(perp), cy - gap*math.sin(perp)
        wx1, wy1  = cx + gap*math.cos(perp), cy + gap*math.sin(perp)
        ax.plot([wx0, wx1], [wy0, wy1], [z, z],
                color=cg, linewidth=3, solid_capstyle='round', zorder=5)
        for sign in (-1, 1):
            fx0 = cx + sign*gap*math.cos(perp)
            fy0 = cy + sign*gap*math.sin(perp)
            fx1 = fx0 + flen*math.cos(angle)
            fy1 = fy0 + flen*math.sin(angle)
            ax.plot([fx0, fx1], [fy0, fy1], [z, z],
                    color=cg, linewidth=5, solid_capstyle='round', zorder=5)
            ax.scatter([fx1], [fy1], [z], color=cg, s=50, zorder=6, depthshade=False)
        ax.quiver(cx, cy, z, 35*math.cos(angle), 35*math.sin(angle), 0,
                  color=AXIS_COLORS[3], linewidth=2,
                  arrow_length_ratio=0.35, alpha=0.9)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = SCARAControlPanel()
    app.run()
