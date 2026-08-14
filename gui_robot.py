"""
SCARA Control Panel Pro — Contrôle manuel + Asservissement visuel
Fusion de gui_robot.py et main_phase3.py
"""

import customtkinter as ctk
import tkinter as tk
import numpy as np
import queue
import threading
import time
import math
import os
import sys
import json
from datetime import datetime
import snap7
from snap7.util import set_real, set_bool, get_bool, get_real

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
        from robot_transform import make_transform_from_geometry, save_transform
        _VS_AVAIL = True
    except Exception as _e:
        _VS_ERR = str(_e)

# ── Thème ─────────────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ── Paramètres robot ──────────────────────────────────────────────────────────
L1, L2 = 150, 180
J1_SCALE = 25000 / math.radians(90)
J2_SCALE = 7500 / math.radians(90)
Z_SCALE  = 95000 / 55.0
J4_SCALE = 2000  / math.radians(180)

AXIS_COLORS  = ['#e94560', '#ff8c42', '#00d4ff', '#a855f7']
AXIS_NAMES   = ["J1 — Épaule", "J2 — Coude", "Z — Hauteur", "J4 — Orient."]
SPEED_NAMES  = ["J1", "J2", "Z", "J4"]
LIMITS       = [(-16000., 25000.), (-11000., 11000.),
                (-6000., 9500.), (-2000., 2300.)]
INIT_POS     = [0.0, 0.0, 0.0, 0.0]
INIT_SPEEDS  = [22000., 25000., 22000., 1800.]
SPEED_LIMITS = [(500, 50000), (500, 50000), (500, 50000), (100, 5000)]
BG_3D        = '#0d1117'
CFG_PATH     = os.path.join(_HERE, 'gui_robot_config.json')

# Ordre attendu par la PLC: J1, Z, J2, J4
PLC_AXIS_ORDER = [0, 2, 1, 3]
PLC_TO_UI_AXIS_ORDER = [0, 2, 1, 3]

def enc_to_physical(enc):
    return (enc[0]/J1_SCALE, enc[1]/J2_SCALE, enc[2]/Z_SCALE, enc[3]/J4_SCALE)


# ── Dialogue de calibration caméra ────────────────────────────────────────────
class CalibrationDialog(ctk.CTkToplevel):
    """Fenêtre de calibration de la caméra par damier."""

    # Drapeaux OpenCV : normalisation + seuillage adaptatif
    # FAST_CHECK volontairement absent : trop de faux-négatifs sur certains éclairages
    _FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH
              | cv2.CALIB_CB_NORMALIZE_IMAGE)

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

    @classmethod
    def _find_board(cls, gray, enhanced, board):
        """Essaie plusieurs combinaisons de drapeaux pour trouver le damier."""
        candidates = [
            (enhanced, cls._FLAGS),
            (gray,     cls._FLAGS),
            (enhanced, cv2.CALIB_CB_ADAPTIVE_THRESH),
            (gray,     cv2.CALIB_CB_ADAPTIVE_THRESH),
            (enhanced, cv2.CALIB_CB_NORMALIZE_IMAGE),
            (gray,     0),
        ]
        for img, flags in candidates:
            found, corners = cv2.findChessboardCorners(img, board, flags)
            if found:
                return True, corners
        return False, None

    def _poll(self):
        if not self._running or self._cap is None:
            return
        ret, frame = self._cap.read()
        if ret:
            board = self._board
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            enhanced = self._preprocess(gray)
            found, corners = self._find_board(gray, enhanced, board)
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
        found, _ = self._find_board(gray, enhanced, board)
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
                ok, corners = self._find_board(gray, enhanced, board)
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


class RobotTransformDialog(ctk.CTkToplevel):
    """Réglage simplifié de la transformation caméra → robot."""

    def __init__(self, parent):
        super().__init__(parent)
        self.title("Calibration caméra → robot")
        self.geometry("460x500")
        self.resizable(True, True)
        self.minsize(400, 480)
        self.lift()
        self.focus_force()
        self.protocol("WM_DELETE_WINDOW", self.destroy)

        self._tx = tk.StringVar(value="-290")
        self._ty = tk.StringVar(value="220")
        self._tz = tk.StringVar(value="280")
        self._rz = tk.StringVar(value="90")
        self._ry = tk.StringVar(value="0")
        self._rx = tk.StringVar(value="0")

        self._build()

    def _build(self):
        root = ctk.CTkFrame(self, corner_radius=10)
        root.pack(fill="both", expand=True, padx=10, pady=10)

        ctk.CTkLabel(
            root,
            text="TRANSFORMATION CAMÉRA → ROBOT",
            font=ctk.CTkFont("Segoe UI", 13, "bold"),
            text_color="#00d4ff",
        ).pack(pady=(14, 8))

        ctk.CTkLabel(
            root,
            text="Saisir une estimation géométrique (mm et degrés).",
            font=ctk.CTkFont("Segoe UI", 10),
            text_color="gray",
        ).pack(pady=(0, 8))

        grid = ctk.CTkFrame(root, fg_color="transparent")
        grid.pack(fill="x", padx=14, pady=6)

        rows = [
            ("Tx (mm)", self._tx), ("Ty (mm)", self._ty), ("Tz (mm)", self._tz),
            ("Rz (°)", self._rz), ("Ry (°)", self._ry), ("Rx (°)", self._rx),
        ]
        for i, (lbl, var) in enumerate(rows):
            ctk.CTkLabel(grid, text=lbl, font=ctk.CTkFont("Segoe UI", 10), text_color="gray"
                         ).grid(row=i, column=0, sticky="w", pady=4)
            ctk.CTkEntry(grid, textvariable=var, width=120, font=ctk.CTkFont("Consolas", 11)
                         ).grid(row=i, column=1, sticky="w", padx=(8, 0), pady=4)

        self._status = ctk.CTkLabel(root, text="", font=ctk.CTkFont("Segoe UI", 9), text_color="gray")
        self._status.pack(pady=(2, 8))

        btns = ctk.CTkFrame(root, fg_color="transparent")
        btns.pack(fill="x", padx=14, pady=(4, 12))
        btns.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkButton(
            btns,
            text="Enregistrer cam_to_robot.npz",
            fg_color="#0d1a3a",
            hover_color="#1a2a5a",
            text_color="#00d4ff",
            font=ctk.CTkFont("Segoe UI", 11, "bold"),
            command=self._save,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 5))

        ctk.CTkButton(
            btns,
            text="Fermer",
            fg_color="#2a0d0d",
            hover_color="#4a1a1a",
            text_color="#ff4444",
            font=ctk.CTkFont("Segoe UI", 11, "bold"),
            command=self.destroy,
        ).grid(row=0, column=1, sticky="ew", padx=(5, 0))

    def _save(self):
        try:
            tx = float(self._tx.get()) / 1000.0
            ty = float(self._ty.get()) / 1000.0
            tz = float(self._tz.get()) / 1000.0
            rz = float(self._rz.get())
            ry = float(self._ry.get())
            rx = float(self._rx.get())

            # Import direct pour être indépendant de _VS_AVAIL
            sys.path.insert(0, _PHASE2)
            from robot_transform import make_transform_from_geometry as _mkT, save_transform as _saveT

            tf = _mkT(translation_m=[tx, ty, tz], rotation_deg=[rz, ry, rx])
            os.makedirs(_CALIB_DIR, exist_ok=True)
            path = os.path.join(_CALIB_DIR, 'cam_to_robot.npz')
            _saveT(tf, path)
            self._status.configure(
                text=f"✓ Enregistré\nTx={self._tx.get()} Ty={self._ty.get()} Tz={self._tz.get()} mm\nRz={self._rz.get()}° Ry={self._ry.get()}° Rx={self._rx.get()}°",
                text_color="#00c851")
        except Exception as exc:
            self._status.configure(text=f"✗ {str(exc)[:80]}", text_color="#ff4444")


# ── Application principale ────────────────────────────────────────────────────
class SCARAControlPanel:

    def __init__(self):
        self.root = ctk.CTk()
        self.root.title("SCARA Control Panel Pro")
        self.root.geometry("1220x820")
        self.root.minsize(1040, 720)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # PLC
        self.plc         = snap7.client.Client()
        self.ip          = '192.168.0.10'
        self.connected   = False
        self.pince_state = False
        self.send_timer  = None
        self.is_sending  = False
        self._syncing_positions = False
        self._plc_lock = threading.Lock()
        self._poll_id = None
        self._reconnect_id = None
        self._auto_reconnect = True
        self._estop = False
        self._manual_edit_until = 0.0
        self._homed = False   # True une fois que l'opérateur a confirmé le homing

        # Variables manuel
        self.pos_vars = [tk.DoubleVar(value=v) for v in INIT_POS]
        self.pos_disp = [tk.StringVar(value=f"{v:.1f}") for v in INIT_POS]
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
        self._vs_marker_sz = tk.StringVar(value="0.035")
        self._vs_work_z    = tk.StringVar(value="0.80")
        self._vs_work_z_robot = tk.StringVar(value="0.0")   # Z cible en repère robot (m)
        self._vs_gain         = tk.DoubleVar(value=0.5)
        self._vs_adaptive     = tk.BooleanVar(value=True)
        self._vs_speed_ratio  = tk.DoubleVar(value=0.20)          # fraction vitesse manuelle
        # Pas max par trame en unités encodeur, ordre PLC [J1, Z, J2, J4]
        self._vs_max_step_vars = [tk.StringVar(value=v) for v in ["100", "50", "100", "30"]]
        self._vs_last_plc_pos  = None   # position envoyée au dernier cycle VS
        self._vs_yolo_mdl      = tk.StringVar(value="yolov8n.pt")
        self._vs_ft_x      = tk.StringVar(value="350")
        self._vs_ft_y      = tk.StringVar(value="0")
        self._vs_ft_z      = tk.StringVar(value="-150")
        self._ip_var       = tk.StringVar(value=self.ip)

        self._preset_slots = {
            "P1": INIT_POS.copy(),
            "P2": INIT_POS.copy(),
            "P3": INIT_POS.copy(),
        }
        self._log_lines = []

        self._load_config()
        self._build_ui()
        self._update_3d()
        # Rappel homing au démarrage (après que la fenêtre soit prête)
        self.root.after(500, self._prompt_homing)

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

    def _prompt_homing(self):
        """Affiche un bandeau d'avertissement homing et bloque les envois."""
        self._homing_bar = ctk.CTkFrame(
            self.root, fg_color="#3a1a00", corner_radius=0, height=38)
        self._homing_bar.grid(row=2, column=0, sticky="ew")
        self._homing_bar.grid_propagate(False)
        self._homing_bar.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(
            self._homing_bar,
            text="⚠  Confirmez que le homing physique a été réalisé avant d'envoyer des commandes",
            font=ctk.CTkFont("Segoe UI", 10, "bold"),
            text_color="#ffbb33",
        ).grid(row=0, column=0, padx=18, pady=6, sticky="w")
        ctk.CTkButton(
            self._homing_bar,
            text="✔  Homing effectué — Activer les commandes",
            height=28,
            fg_color="#0d2a18",
            hover_color="#1a4a2a",
            text_color="#00c851",
            font=ctk.CTkFont("Segoe UI", 10, "bold"),
            command=self._confirm_homing,
        ).grid(row=0, column=2, padx=18, pady=4)

    def _confirm_homing(self):
        self._homed = True
        if hasattr(self, "_homing_bar"):
            self._homing_bar.destroy()
        self._log("Homing confirmé par l'opérateur — commandes débloquées", "INFO")
        self.lbl_send.configure(text="✓  Homing confirmé")

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

        net = ctk.CTkFrame(bar, fg_color="transparent")
        net.grid(row=0, column=2, sticky="w", padx=8)
        ctk.CTkLabel(net, text="PLC:",
                 font=ctk.CTkFont("Consolas", 9),
                 text_color="gray").pack(side="left", padx=(0, 6))
        ctk.CTkEntry(net, textvariable=self._ip_var, width=120,
                 font=ctk.CTkFont("Consolas", 10)
                 ).pack(side="left", padx=(0, 6))
        ctk.CTkButton(net, text="Appliquer", width=78, height=28,
                  font=ctk.CTkFont("Segoe UI", 9, "bold"),
                  command=self._apply_ip).pack(side="left")

        self._lbl_ip = ctk.CTkLabel(bar, text=f"IP active: {self.ip}",
                                    font=ctk.CTkFont("Consolas", 9),
                                    text_color="gray")
        self._lbl_ip.grid(row=0, column=2, sticky="w", padx=(240, 8))

        right = ctk.CTkFrame(bar, fg_color="transparent")
        right.grid(row=0, column=3, padx=14, sticky="e")

        self.lbl_status = ctk.CTkLabel(right, text="●  DÉCONNECTÉ",
                                       font=ctk.CTkFont("Segoe UI", 11, "bold"),
                                       text_color="#ff4444")
        self.lbl_status.pack(side="left")

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
    # MODE MANUEL : vue compacte + curseurs
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_manual_mode(self, parent):
        self._build_motion_panel(parent)
        self._build_ops_panel(parent)

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

    def _build_motion_panel(self, parent):
        panel = ctk.CTkFrame(parent, corner_radius=12)
        panel.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        panel.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(panel, text="MOUVEMENT",
                     font=ctk.CTkFont("Segoe UI", 12, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(12, 6))

        self._build_coord_bar(panel, row=1)

        strip = ctk.CTkFrame(panel, fg_color="transparent")
        strip.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 8))
        strip.grid_columnconfigure((0, 1, 2), weight=1)
        ctk.CTkButton(strip, text="Home", height=30,
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      command=self._go_home).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ctk.CTkButton(strip, text="Sync PLC", height=30,
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      command=self.sync_axes_from_plc).grid(row=0, column=1, sticky="ew", padx=4)
        ctk.CTkButton(strip, text="Stop Move", height=30,
                      fg_color="#3a1f1f", hover_color="#4f2b2b",
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      command=self._stop_motion_soft).grid(row=0, column=2, sticky="ew", padx=(4, 0))

        axes = ctk.CTkFrame(panel, fg_color="transparent")
        axes.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 4))
        axes.grid_columnconfigure((0, 1), weight=1)
        for i in range(4):
            self._build_axis_card(axes, i, row=i // 2, column=i % 2)

        pw = ctk.CTkFrame(panel, fg_color="transparent")
        pw.grid(row=4, column=0, sticky="ew", padx=16, pady=(10, 4))
        pw.grid_columnconfigure(0, weight=1)
        self.btn_pince = ctk.CTkButton(
            pw, text="⊙  PINCE : OUVERTE", height=52,
            fg_color="#2e2410", hover_color="#4a3a18", text_color="#ffbb33",
            font=ctk.CTkFont("Segoe UI", 14, "bold"),
            corner_radius=10, command=self.toggle_pince)
        self.btn_pince.grid(row=0, column=0, sticky="ew")

        self._build_speed_panel(panel, row=5)

        self.lbl_send = ctk.CTkLabel(panel, text="",
                                     font=ctk.CTkFont("Segoe UI", 9),
                                     text_color="#00d4ff")
        self.lbl_send.grid(row=6, column=0, pady=(4, 10))

    def _build_ops_panel(self, parent):
        panel = ctk.CTkFrame(parent, corner_radius=12)
        panel.grid(row=0, column=1, sticky="nsew", padx=(7, 0))
        panel.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(panel, text="OPÉRATIONS",
                     font=ctk.CTkFont("Segoe UI", 12, "bold"),
                     text_color="#00d4ff").grid(row=0, column=0, pady=(12, 6))

        status = ctk.CTkFrame(panel, corner_radius=10)
        status.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 8))
        status.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(status, text="PLC", font=ctk.CTkFont("Segoe UI", 9, "bold"),
                     text_color="gray").grid(row=0, column=0, padx=12, pady=(10, 2), sticky="w")
        self._conn_badge = ctk.CTkLabel(status, text="● DÉCONNECTÉ",
                                        font=ctk.CTkFont("Segoe UI", 11, "bold"),
                                        text_color="#ff4444")
        self._conn_badge.grid(row=1, column=0, padx=12, pady=(0, 10), sticky="w")
        self._ops_hint = ctk.CTkLabel(status, text="Mode manuel prêt",
                                      font=ctk.CTkFont("Segoe UI", 10),
                                      text_color="gray")
        self._ops_hint.grid(row=1, column=1, padx=12, pady=(0, 10), sticky="e")

        self._build_presets_panel(panel, row=2)
        self._build_log_panel(panel, row=3)

        footer = ctk.CTkFrame(panel, fg_color="transparent")
        footer.grid(row=4, column=0, sticky="ew", padx=16, pady=(10, 12))
        footer.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(footer, text="Connecter PLC", height=36,
                      fg_color="#0d2a18", hover_color="#1a4a2a",
                      text_color="#00c851",
                      font=ctk.CTkFont("Segoe UI", 10, "bold"),
                      command=self.connect_plc).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self._btn_estop = ctk.CTkButton(footer, text="E-STOP", height=36,
                        fg_color="#6a0000", hover_color="#8a0000",
                        text_color="#ffffff",
                        font=ctk.CTkFont("Segoe UI", 10, "bold"),
                        command=self._toggle_estop)
        self._btn_estop.grid(row=0, column=1, sticky="ew", padx=(4, 0))

    def _build_axis_card(self, parent, i, row, column=0):
        card = ctk.CTkFrame(parent, corner_radius=10)
        card.grid(row=row, column=column, sticky="ew", padx=6, pady=4)
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
        ctk.CTkButton(top, text="−", width=24, height=20,
                  fg_color="#1f1f2f", hover_color="#2a2a3a",
                  font=ctk.CTkFont("Segoe UI", 10, "bold"),
                  command=lambda idx=i: self._jog_axis(idx, -200)
                  ).grid(row=0, column=3, padx=(8, 2))
        ctk.CTkButton(top, text="+", width=24, height=20,
                  fg_color="#1f1f2f", hover_color="#2a2a3a",
                  font=ctk.CTkFont("Segoe UI", 10, "bold"),
                  command=lambda idx=i: self._jog_axis(idx, 200)
                  ).grid(row=0, column=4)

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

    def _build_presets_panel(self, parent, row):
        frame = ctk.CTkFrame(parent, corner_radius=10)
        frame.grid(row=row, column=0, sticky="ew", padx=16, pady=(6, 2))
        frame.grid_columnconfigure((0, 1, 2), weight=1)
        ctk.CTkLabel(frame, text="PRÉSETS",
                     font=ctk.CTkFont("Segoe UI", 9, "bold"),
                     text_color="gray").grid(row=0, column=0, columnspan=3, sticky="w", padx=10, pady=(5, 2))

        for i, name in enumerate(["P1", "P2", "P3"]):
            slot = ctk.CTkFrame(frame, corner_radius=7)
            slot.grid(row=1, column=i, sticky="ew", padx=4, pady=(2, 6))
            ctk.CTkLabel(slot, text=name,
                         font=ctk.CTkFont("Consolas", 10, "bold"), text_color="#00d4ff"
                         ).pack(pady=(4, 1))
            ctk.CTkButton(slot, text="Mémoriser", height=24,
                          font=ctk.CTkFont("Segoe UI", 9),
                          command=lambda n=name: self._save_preset(n)
                          ).pack(fill="x", padx=6, pady=2)
            ctk.CTkButton(slot, text="Rappeler", height=24,
                          font=ctk.CTkFont("Segoe UI", 9),
                          command=lambda n=name: self._load_preset(n)
                          ).pack(fill="x", padx=6, pady=(0, 5))

    def _build_log_panel(self, parent, row):
        frame = ctk.CTkFrame(parent, corner_radius=10)
        frame.grid(row=row, column=0, sticky="nsew", padx=16, pady=(4, 0))
        frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(frame, text="JOURNAL",
                     font=ctk.CTkFont("Segoe UI", 9, "bold"),
                     text_color="gray").grid(row=0, column=0, sticky="w", padx=10, pady=(5, 2))
        self._log_box = ctk.CTkTextbox(frame, height=92, corner_radius=8,
                                       font=("Consolas", 9))
        self._log_box.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        self._log_box.insert("1.0", "[INFO] Journal initialisé\n")
        self._log_box.configure(state="disabled")

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
            ("Cible X,Y (mm)", AXIS_COLORS[0]), ("Cible r,Z (mm)", AXIS_COLORS[2]),
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
        parent.grid_rowconfigure(13, weight=1)

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
        ctk.CTkButton(src, text="📐 Cam→Robot", width=108, height=28,
                  fg_color="#1b2436", hover_color="#273249",
                  text_color="#9fd0ff",
                  font=ctk.CTkFont("Segoe UI", 9, "bold"),
                  command=self._open_robot_transform
                  ).grid(row=0, column=3, padx=(8, 0))

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
            ("Plan de travail Z cam (m) :", self._vs_work_z),
            ("Z cible robot (m, NaN=auto) :", self._vs_work_z_robot),
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

        self._build_vs_security(parent, row=10)

        # ── Bouton START / STOP ──────────────────────────────────────────────
        btn_wrap = ctk.CTkFrame(parent, fg_color="transparent")
        btn_wrap.grid(row=14, column=0, sticky="ew", padx=16, pady=(14, 4))
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
        self._lbl_vs_status.grid(row=15, column=0, pady=(4, 10))

    def _build_vs_security(self, parent, row: int):
        """Panneau sécurités VS : vitesse réduite + limitation pas/trame."""
        self._sep(parent, "SÉCURITÉS VS", row=row)

        # ── Ratio de vitesse ─────────────────────────────────────────────────
        sr = ctk.CTkFrame(parent, fg_color="transparent")
        sr.grid(row=row + 1, column=0, sticky="ew", padx=16, pady=3)
        sr.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(sr, text="Vitesse VS :",
                     font=ctk.CTkFont("Segoe UI", 9),
                     text_color="gray").grid(row=0, column=0, padx=(0, 8))
        ctk.CTkSlider(sr, from_=0.05, to=1.0, variable=self._vs_speed_ratio,
                      button_color="#ff8c42", progress_color="#ff8c42",
                      height=14).grid(row=0, column=1, sticky="ew")
        self._lbl_vs_spd = ctk.CTkLabel(sr, text="20%",
                                         font=ctk.CTkFont("Consolas", 10),
                                         text_color="#ff8c42", width=42)
        self._lbl_vs_spd.grid(row=0, column=2, padx=(6, 0))
        self._vs_speed_ratio.trace_add("write", lambda *_: self._lbl_vs_spd.configure(
            text=f"{self._vs_speed_ratio.get() * 100:.0f}%"))

        # ── Pas max par trame (enc/frame) — anti-embalement ──────────────────
        ms = ctk.CTkFrame(parent, fg_color="transparent")
        ms.grid(row=row + 2, column=0, sticky="ew", padx=16, pady=(2, 4))
        ctk.CTkLabel(ms, text="Pas max/trame :",
                     font=ctk.CTkFont("Segoe UI", 8),
                     text_color="gray").pack(side="left", padx=(0, 4))
        # Ordre PLC [J1, Z, J2, J4] — couleurs mappées sur l'ordre UI
        _plc_colors = [AXIS_COLORS[0], AXIS_COLORS[2], AXIS_COLORS[1], AXIS_COLORS[3]]
        for lbl, var, col in zip(["J1", "Z", "J2", "J4"],
                                  self._vs_max_step_vars, _plc_colors):
            ctk.CTkLabel(ms, text=lbl,
                         font=ctk.CTkFont("Consolas", 9),
                         text_color=col).pack(side="left", padx=(6, 1))
            ctk.CTkEntry(ms, textvariable=var, width=46,
                         font=ctk.CTkFont("Consolas", 9)).pack(side="left", padx=(0, 2))

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

    def _open_robot_transform(self):
        dlg = RobotTransformDialog(self.root)
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
        if self._estop:
            self._log("E-STOP actif, démarrage VS bloqué", "WARN")
            self._lbl_vs_status.configure(text="E-STOP actif", text_color="#ff4444")
            return
        if not self.connected:
            self._log("VS démarré sans PLC connectée: vision OK, robot immobile", "WARN")
        self._vs_running = True
        self._btn_vs.configure(text="■  ARRÊTER",
                               fg_color="#2a0d0d", hover_color="#4a1a1a",
                               text_color="#ff4444")
        self._lbl_vs_status.configure(text="Initialisation...",
                                      text_color="#ffbb33")
        self._log(
            f"Start VS cam={self._vs_cam_idx.get()} mode={self._vs_method.get()} gain={self._vs_gain.get():.2f} adaptive={self._vs_adaptive.get()}",
            "INFO",
        )
        self._vs_last_plc_pos = None
        # Pré-charger Z depuis la PLC pour éviter d'envoyer Z=0 au premier cycle
        if self.connected:
            try:
                _raw = self._plc_read_area(0x83, 0, 100, 16)
                self._vs_last_plc_pos = [get_real(_raw, i * 4) for i in range(4)]
            except Exception:
                pass
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
        last_state = None
        no_target_frames = 0
        target_was_visible = False
        last_diag_t = time.time()
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
                work_plane_z_robot = (None if self._vs_work_z_robot.get().strip().lower() in ('nan', '', 'none', 'auto')
                                      else float(self._vs_work_z_robot.get())),
                yolo_model   = self._vs_yolo_mdl.get() if actual_method == "yolo" else None,
                force_target = ft,
                dt           = 1.0 / 30,
            )
            pipeline.ctrl.tune(gain=self._vs_gain.get())
            pipeline.ctrl.adaptive = bool(self._vs_adaptive.get())
            self._vs_pipeline = pipeline

            # ── Paramètres physiques du robot réel ─────────────────────────
            # ScaraParams défaut = 300/160 mm ; robot réel = L1/L2 = 150/110 mm.
            # d3/d4 sont des offsets Z du modèle DH qui ne correspondent pas
            # à ce robot ; on les met à 0 pour que d2 = hauteur EE directement.
            pipeline.params.a2 = L1 / 1000.0          # 0.150 m
            pipeline.params.a3 = L2 / 1000.0          # 0.110 m
            pipeline.params.d3 = 0.0
            pipeline.params.d4 = 0.0
            pipeline.params.d2_base = 0.0
            pipeline.params.q_min = np.array([
                LIMITS[0][0] / J1_SCALE,           # J1 min  (rad)
                LIMITS[2][0] / Z_SCALE / 1000.0,   # Z  min  (m)
                LIMITS[1][0] / J2_SCALE,           # J2 min  (rad)
                -3.142,
            ])
            pipeline.params.q_max = np.array([
                LIMITS[0][1] / J1_SCALE,           # J1 max  (rad)
                LIMITS[2][1] / Z_SCALE / 1000.0,   # Z  max  (m)
                LIMITS[1][1] / J2_SCALE,           # J2 max  (rad)
                3.142,
            ])
            # VSController garde une référence sur le même objet ScaraParams.
            # Forcer la mise à jour au cas où ce serait une copie.
            if hasattr(pipeline.ctrl, 'params'):
                pipeline.ctrl.params = pipeline.params
            self.root.after(0, lambda a2=pipeline.params.a2, a3=pipeline.params.a3: self._log(
                f"Params robot: a2={a2*1000:.0f}mm a3={a3*1000:.0f}mm d3=0 d4=0", "INFO"))
            # ───────────────────────────────────────────────────────────────

            # Initialisation depuis la PLC pour aligner l'état interne.
            if self.connected:
                self._sync_vs_pipeline_from_plc(pipeline)
                q0 = pipeline.q_current
                self.root.after(0, lambda q=q0: self._log(
                    f"VS init q: J1={np.degrees(q[0]):.1f}° J2={np.degrees(q[2]):.1f}° "
                    f"Z={q[1]*1000:.1f}mm J4={np.degrees(q[3]):.1f}°", "INFO"))
            else:
                # Fallback : utiliser la position affichée dans l'UI (sliders)
                try:
                    ui = [float(v.get()) for v in self.pos_vars]  # [J1,J2,Z,J4] enc
                    pipeline.q_current = np.array([
                        ui[0] / J1_SCALE,
                        ui[2] / Z_SCALE / 1000.0,
                        ui[1] / J2_SCALE,
                        ui[3] / J4_SCALE,
                    ])
                except Exception:
                    pass

            cap = cv2.VideoCapture(cam_idx)
            if not cap.isOpened():
                self.root.after(0, lambda: self._log(f"Caméra {cam_idx} introuvable", "ERR"))
                self.root.after(0, lambda: self._lbl_vs_status.configure(
                    text=f"✗ Caméra {cam_idx} introuvable", text_color="#ff4444"))
                return

            self.root.after(0, lambda: self._lbl_vs_status.configure(
                text="● En cours", text_color="#00c851"))

            while self._vs_running:
                ret, frame = cap.read()
                if not ret:
                    self.root.after(0, lambda: self._log("Lecture caméra impossible (ret=False)", "ERR"))
                    break

                annotated, cmd, state = pipeline.process_frame(frame)

                target_visible = pipeline._dbg_t_target is not None
                if target_visible:
                    no_target_frames = 0
                    if not target_was_visible:
                        target_was_visible = True
                        self.root.after(0, lambda: self._log("Cible détectée par le pipeline VS", "INFO"))
                else:
                    no_target_frames += 1
                    if target_was_visible:
                        target_was_visible = False
                        self.root.after(0, lambda: self._log("Perte de cible visuelle", "WARN"))
                    now = time.time()
                    if no_target_frames > 45 and now - last_diag_t > 2.0:
                        last_diag_t = now
                        self.root.after(0, lambda: self._log(
                            "Aucune cible valide (ArUco/YOLO) -> pas de commande mouvement",
                            "WARN",
                        ))

                # Avertissement si la cible est hors de la portée du robot
                if pipeline._dbg_t_target is not None:
                    _tr = float(np.linalg.norm(pipeline._dbg_t_target[:2])) * 1000
                    if _tr > float(L1 + L2) * 1.05:
                        now = time.time()
                        if now - last_diag_t > 3.0:
                            last_diag_t = now
                            self.root.after(0, lambda r=_tr: self._log(
                                f"⚠ Cible hors portée: r={r:.0f}mm (max={(L1+L2):.0f}mm) — vérifier cam→robot",
                                "WARN"))

                if not self.connected:
                    now = time.time()
                    if now - last_diag_t > 2.0:
                        last_diag_t = now
                        self.root.after(0, lambda: self._log(
                            "PLC non connectée pendant VS: mouvement robot désactivé",
                            "WARN",
                        ))

                # ── Envoi PLC ──────────────────────────────────────────────
                if self.connected and state != PipelineState.EMERGENCY and (cmd is not None or state == PipelineState.PRE_APPROACH):
                    try:
                        if self._estop:
                            continue
                        q = pipeline.q_current
                        plc_pos = list(self._pipeline_q_to_plc_pos(q))

                        # Z gelé à la valeur PLC courante — le VS ne commande que XY
                        if self._vs_last_plc_pos is not None:
                            plc_pos[1] = self._vs_last_plc_pos[1]

                        # ── Sécurité 1 : clamp aux limites mécaniques ──────
                        # Ordre PLC : [J1, Z, J2, J4]
                        _plc_lim = [LIMITS[0], LIMITS[2], LIMITS[1], LIMITS[3]]
                        for _i in range(4):
                            plc_pos[_i] = float(np.clip(
                                plc_pos[_i], _plc_lim[_i][0], _plc_lim[_i][1]))

                        # ── Sécurité 2 : limitation du pas max par trame ───
                        try:
                            _max_s = [max(1.0, float(self._vs_max_step_vars[j].get()))
                                      for j in range(4)]
                        except Exception:
                            _max_s = [100.0, 50.0, 100.0, 30.0]
                        if self._vs_last_plc_pos is not None:
                            for _i in range(4):
                                _d = plc_pos[_i] - self._vs_last_plc_pos[_i]
                                if abs(_d) > _max_s[_i]:
                                    plc_pos[_i] = (self._vs_last_plc_pos[_i]
                                                   + _max_s[_i] * float(np.sign(_d)))
                        self._vs_last_plc_pos = list(plc_pos)
                        # Resync q_current avec ce qui a été réellement commandé
                        # (après clamp + rate-limit). Le pipeline avance en phase
                        # avec ce qui a vraiment été envoyé au lieu d’intégrer
                        # librement et diverger de la réalité.
                        _q_sent = self._plc_pos_to_pipeline_q(plc_pos)
                        if np.all(np.isfinite(_q_sent)):
                            pipeline.q_current = _q_sent

                        data = bytearray(32)
                        set_real(data,  0, plc_pos[0])
                        set_real(data,  4, plc_pos[1])
                        set_real(data,  8, plc_pos[2])
                        set_real(data, 12, plc_pos[3])

                        # ── Sécurité 3 : vitesse réduite en mode VS ────────
                        _ratio = max(0.05, min(1.0, float(self._vs_speed_ratio.get())))
                        logical_vit = [float(v.get()) for v in self.vit_vars]
                        plc_vit = [logical_vit[i] * _ratio for i in PLC_AXIS_ORDER]
                        set_real(data, 16, plc_vit[0])
                        set_real(data, 20, plc_vit[1])
                        set_real(data, 24, plc_vit[2])
                        set_real(data, 28, plc_vit[3])
                        self._plc_write_area(0x83, 0, 100, data)

                        gripper_close = (
                            pipeline.sequencer.gripper.state.name
                            in ("CLOSED", "CLOSING"))
                        m34 = self._plc_read_area(0x83, 0, 34, 1)
                        set_bool(m34, 0, 5, gripper_close)
                        self._plc_write_area(0x83, 0, 34, m34)

                        if state not in (PipelineState.SEARCHING,
                                         PipelineState.EMERGENCY):
                            m50 = bytearray(1)
                            set_bool(m50, 0, 0, True)
                            self._plc_write_area(0x83, 0, 50, m50)
                            time.sleep(0.01)
                            set_bool(m50, 0, 0, False)
                            self._plc_write_area(0x83, 0, 50, m50)
                    except Exception as exc:
                        self.root.after(0, lambda e=str(exc): self._lbl_vs_status.configure(
                            text=f"✗ PLC write: {e[:80]}", text_color="#ff4444"))
                        self.root.after(0, lambda e=str(exc): self._log(f"VS PLC write error: {e}", "ERR"))

                # ── Mise à jour barre d état ───────────────────────────────
                try:
                    bgr  = STATE_COLORS.get(state, (100, 100, 100))
                    hcol = f"#{bgr[2]:02x}{bgr[1]:02x}{bgr[0]:02x}"
                    e_mm = f"{pipeline.errors[-1].norm_t_mm:.1f}" if pipeline.errors else "—"
                    _tgt = pipeline._dbg_t_target
                    if _tgt is not None:
                        _tmm = _tgt * 1000
                        xy_s = f"{_tmm[0]:+.0f}, {_tmm[1]:+.0f}"
                        rz_s = f"r={float(np.linalg.norm(_tmm[:2])):.0f} z={_tmm[2]:+.0f}"
                    else:
                        xy_s, rz_s = "—", "—"
                    self.root.after(0, lambda s=state.name, e=e_mm, xy=xy_s, z=rz_s, c=hcol: (
                        self._vs_bar_labels[0].configure(text=s, text_color=c),
                        self._vs_bar_labels[1].configure(text=e),
                        self._vs_bar_labels[2].configure(text=xy),
                        self._vs_bar_labels[3].configure(text=z),
                    ))
                except Exception:
                    pass

                if state != last_state:
                    last_state = state
                    self.root.after(0, lambda s=state.name: self._log(f"État VS -> {s}", "INFO"))
                    if state == PipelineState.SEARCHING:
                        self.root.after(0, lambda: self._lbl_vs_status.configure(
                            text="● En cours — recherche cible", text_color="#ffbb33"))
                    elif state == PipelineState.PRE_APPROACH:
                        self.root.after(0, lambda: self._lbl_vs_status.configure(
                            text="● En cours — pré-approche", text_color="#00d4ff"))
                    elif state == PipelineState.EMERGENCY:
                        self.root.after(0, lambda: self._lbl_vs_status.configure(
                            text="✗ État EMERGENCY", text_color="#ff4444"))
                    else:
                        self.root.after(0, lambda: self._lbl_vs_status.configure(
                            text=f"● En cours — {state.name}", text_color="#00c851"))

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
            self.root.after(0, lambda e=err_msg: self._log(f"Crash VS worker: {e}", "ERR"))
            self.root.after(0, lambda e=err_msg: self._stop_vs(error=e))
        else:
            self.root.after(0, self._stop_vs)
        finally:
            if cap is not None:
                cap.release()
            self.root.after(0, lambda: self._log("Thread VS arrêté", "INFO"))
            self._vs_pipeline = None
            self._vs_running   = False

    @staticmethod
    def _pipeline_q_to_plc_pos(q: np.ndarray):
        """Convert pipeline q [J1_rad, Z_m, J2_rad, J4_rad] to PLC encoder units.
        PLC layout at M100: [J1_enc, Z_enc, J2_enc, J4_enc] (PLC_AXIS_ORDER)."""
        return [
            float(q[0] * J1_SCALE),                # rad → enc (J1)
            float(q[1] * 1000.0 * Z_SCALE),         # m → mm → enc (Z)
            float(q[2] * J2_SCALE),                 # rad → enc (J2)
            float(q[3] * J4_SCALE),                 # rad → enc (J4)
        ]

    @staticmethod
    def _plc_pos_to_pipeline_q(plc_pos):
        """Convert PLC encoder units to pipeline q [J1_rad, Z_m, J2_rad, J4_rad].
        PLC layout at M100: [J1_enc, Z_enc, J2_enc, J4_enc] (PLC_AXIS_ORDER)."""
        return np.array([
            float(plc_pos[0]) / J1_SCALE,             # enc → rad (J1)
            float(plc_pos[1]) / Z_SCALE / 1000.0,     # enc → mm → m (Z)
            float(plc_pos[2]) / J2_SCALE,             # enc → rad (J2)
            float(plc_pos[3]) / J4_SCALE if len(plc_pos) > 3 else 0.0,  # enc → rad (J4)
        ], dtype=float)

    def _sync_vs_pipeline_from_plc(self, pipeline):
        """Resynchronise l'état interne du pipeline avec la position PLC."""
        try:
            raw = self._plc_read_area(0x83, 0, 100, 16)
            plc_pos = [get_real(raw, i * 4) for i in range(4)]

            # Garde-fou: valeurs en unités encodeur (ordre PLC: J1, Z, J2, J4).
            # LIMITS[0]=J1, LIMITS[2]=Z, LIMITS[1]=J2  (indices UI ≠ indices PLC)
            ok = (LIMITS[0][0] <= plc_pos[0] <= LIMITS[0][1] and
                  LIMITS[2][0] <= plc_pos[1] <= LIMITS[2][1] and
                  LIMITS[1][0] <= plc_pos[2] <= LIMITS[1][1])
            if not ok:
                self.root.after(0, lambda p=plc_pos: self._log(
                    f"Sync PLC rejeté (hors limites): J1={p[0]:.0f} Z={p[1]:.0f} J2={p[2]:.0f}", "WARN"))
                return

            q_plc = self._plc_pos_to_pipeline_q(plc_pos)
            if np.all(np.isfinite(q_plc)):
                pipeline.q_current = q_plc
        except Exception as _e:
            self.root.after(0, lambda e=str(_e): self._log(
                f"Sync PLC exception: {e}", "WARN"))

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

    def _apply_ip(self):
        new_ip = self._ip_var.get().strip()
        if not new_ip:
            return
        if self.connected:
            self.disconnect_plc()
        self.ip = new_ip
        self._lbl_ip.configure(text=f"IP active: {self.ip}")
        self._save_config()
        self._log(f"IP PLC mise à jour: {self.ip}")

    def _do_connect(self):
        self.root.after(0, lambda: self.lbl_status.configure(
            text="●  CONNEXION...", text_color="#ffbb33"))
        try:
            self.plc.connect(self.ip, 0, 1)
            self.connected = True
            self.root.after(0, lambda: self.lbl_status.configure(
                text="●  CONNECTÉ", text_color="#00c851"))
            self.root.after(0, lambda: hasattr(self, "_conn_badge") and self._conn_badge.configure(text="● CONNECTÉ", text_color="#00c851"))
            self.root.after(0, lambda: hasattr(self, "_ops_hint") and self._ops_hint.configure(text="Axes synchronisés", text_color="#00c851"))
            self.root.after(0, self.sync_axes_from_plc)
            self.root.after(0, self.read_pince)
            self.root.after(0, self._start_plc_polling)
            self.root.after(0, lambda: self._log(f"PLC connectée: {self.ip}"))
        except Exception:
            self.root.after(0, lambda: self.lbl_status.configure(
                text="●  ERREUR", text_color="#ff4444"))
            self.root.after(0, lambda: hasattr(self, "_conn_badge") and self._conn_badge.configure(text="● ERREUR", text_color="#ff4444"))
            self.root.after(0, lambda: self._log("Connexion PLC échouée", "ERR"))
            self.root.after(0, self._schedule_reconnect)

    def disconnect_plc(self):
        if self.connected:
            try:
                self.plc.disconnect()
            except Exception:
                pass
            self.connected = False
        if self._reconnect_id is not None:
            self.root.after_cancel(self._reconnect_id)
            self._reconnect_id = None
        self.lbl_status.configure(text="●  DÉCONNECTÉ", text_color="#ff4444")
        if hasattr(self, "_conn_badge"):
            self._conn_badge.configure(text="● DÉCONNECTÉ", text_color="#ff4444")
        if hasattr(self, "_ops_hint"):
            self._ops_hint.configure(text="Mode manuel prêt", text_color="gray")
        self._stop_plc_polling()
        self._log("PLC déconnectée")

    def read_pince(self):
        try:
            m34 = self._plc_read_area(0x83, 0, 34, 1)
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
        if self._estop:
            return
        try:
            self.pince_state = not self.pince_state
            m34 = self._plc_read_area(0x83, 0, 34, 1)
            set_bool(m34, 0, 5, self.pince_state)
            self._plc_write_area(0x83, 0, 34, m34)
            self._update_pince_btn()
            self._update_3d()
        except Exception:
            pass

    def on_slider_change(self, _=None):
        if not self._3d_pending:
            self._3d_pending = True
            self.root.after(50, self._scheduled_3d_update)
        if self._syncing_positions:
            return
        # Empêche le polling PLC de réécrire immédiatement les sliders.
        self._manual_edit_until = time.time() + 0.9
        if not self.connected:
            return
        if self._estop:
            self.lbl_send.configure(text="⛔ E-STOP actif")
            return
        if self.send_timer is not None:
            self.root.after_cancel(self.send_timer)
        self.send_timer = self.root.after(200, self._trigger_move)

    def sync_axes_from_plc(self):
        if not self.connected:
            return
        try:
            plc_data = self._plc_read_area(0x83, 0, 100, 16)
            plc_pos = [get_real(plc_data, i * 4) for i in range(4)]
            ui_pos = [plc_pos[i] for i in PLC_TO_UI_AXIS_ORDER]

            self._syncing_positions = True
            for i, value in enumerate(ui_pos):
                self.pos_vars[i].set(value)
            self._update_3d()
            self.lbl_send.configure(text="✓  Axes synchronisés depuis la PLC")
        except Exception:
            self.lbl_send.configure(text="✗  Lecture axes PLC impossible")
        finally:
            self._syncing_positions = False

    def _plc_read_area(self, area, db, start, size):
        with self._plc_lock:
            return self.plc.read_area(area, db, start, size)

    def _plc_write_area(self, area, db, start, data):
        with self._plc_lock:
            self.plc.write_area(area, db, start, data)

    def _start_plc_polling(self):
        self._stop_plc_polling()
        self._poll_id = self.root.after(250, self._poll_plc_state)

    def _stop_plc_polling(self):
        if self._poll_id is not None:
            self.root.after_cancel(self._poll_id)
            self._poll_id = None

    def _poll_plc_state(self):
        if not self.connected:
            self._poll_id = None
            return
        try:
            # Si l'opérateur manipule les sliders, on laisse la consigne locale.
            if time.time() >= self._manual_edit_until:
                plc_data = self._plc_read_area(0x83, 0, 100, 16)
                plc_pos = [get_real(plc_data, i * 4) for i in range(4)]
                ui_pos = [plc_pos[i] for i in PLC_TO_UI_AXIS_ORDER]

                self._syncing_positions = True
                for i, value in enumerate(ui_pos):
                    self.pos_vars[i].set(value)
                self._syncing_positions = False
                self._update_3d()
            self.read_pince()
            self.lbl_send.configure(text="PLC: retour position OK")
        except Exception:
            self._syncing_positions = False
            self.connected = False
            self.lbl_status.configure(text="●  DÉCONNECTÉ", text_color="#ff4444")
            self.lbl_send.configure(text="✗  Perte communication PLC")
            self._log("Perte communication PLC", "ERR")
            self._schedule_reconnect()
            self._poll_id = None
            return

        self._poll_id = self.root.after(250, self._poll_plc_state)

    def _schedule_reconnect(self):
        if not self._auto_reconnect or self.connected:
            return
        if self._reconnect_id is not None:
            return
        self._reconnect_id = self.root.after(5000, self._attempt_reconnect)

    def _attempt_reconnect(self):
        self._reconnect_id = None
        if self.connected:
            return
        self._log("Tentative reconnexion PLC...")
        self.connect_plc()

    def _scheduled_3d_update(self):
        self._3d_pending = False
        self._update_3d()

    def _trigger_move(self):
        if self._estop:
            return
        if not self._homed:
            self.lbl_send.configure(text="⚠  Confirmez le homing avant d'envoyer une commande")
            return
        if not self.is_sending:
            threading.Thread(target=self._send_to_plc, daemon=True).start()

    def _send_to_plc(self):
        self.is_sending = True
        self.root.after(0, lambda: self.lbl_send.configure(text="↑  Envoi en cours..."))
        try:
            if self._estop:
                return
            data = bytearray(32)
            logical_pos = [float(v.get()) for v in self.pos_vars]
            logical_vit = [float(v.get()) for v in self.vit_vars]
            plc_pos = [logical_pos[i] for i in PLC_AXIS_ORDER]
            plc_vit = [logical_vit[i] for i in PLC_AXIS_ORDER]

            for i in range(4):
                set_real(data, i*4,    plc_pos[i])
                set_real(data, 16+i*4, plc_vit[i])
            self._plc_write_area(0x83, 0, 100, data)
            m50 = self._plc_read_area(0x83, 0, 50, 1)
            set_bool(m50, 0, 0, True)
            self._plc_write_area(0x83, 0, 50, m50)
            time.sleep(0.3)
            set_bool(m50, 0, 0, False)
            self._plc_write_area(0x83, 0, 50, m50)
            self.root.after(0, lambda: self.lbl_send.configure(
                text="✓  Commande envoyée"))
        except Exception:
            self.root.after(0, lambda: self.lbl_send.configure(
                text="✗  Erreur d envoi"))
        finally:
            self.is_sending = False

    def _toggle_estop(self):
        if not self._estop:
            self._estop = True
            if self._vs_running:
                self._stop_vs()
            if self.send_timer is not None:
                self.root.after_cancel(self.send_timer)
                self.send_timer = None
            self._btn_estop.configure(text="RESET E-STOP", fg_color="#aa0000", hover_color="#cc0000")
            self.lbl_send.configure(text="⛔ E-STOP actif")
            if hasattr(self, "_ops_hint"):
                self._ops_hint.configure(text="Blocage sécurité actif", text_color="#ffbb33")
            self._stop_motion_soft()
            self._log("E-STOP activé", "WARN")
        else:
            self._estop = False
            self._btn_estop.configure(text="E-STOP", fg_color="#6a0000", hover_color="#8a0000")
            self.lbl_send.configure(text="E-STOP relâché")
            if hasattr(self, "_ops_hint"):
                self._ops_hint.configure(text="Mode manuel prêt", text_color="gray")
            self._log("E-STOP relâché")

    def _stop_motion_soft(self):
        if not self.connected:
            return
        try:
            data = bytearray(32)
            logical_pos = [float(v.get()) for v in self.pos_vars]
            plc_pos = [logical_pos[i] for i in PLC_AXIS_ORDER]
            for i in range(4):
                set_real(data, i * 4, plc_pos[i])
                set_real(data, 16 + i * 4, 0.0)
            self._plc_write_area(0x83, 0, 100, data)
            self.lbl_send.configure(text="■ Mouvement stoppé")
        except Exception:
            self.lbl_send.configure(text="✗ Stop mouvement impossible")

    def _jog_axis(self, axis_idx, delta):
        if self._estop:
            return
        new_val = float(self.pos_vars[axis_idx].get()) + float(delta)
        lo, hi = LIMITS[axis_idx]
        self.pos_vars[axis_idx].set(float(np.clip(new_val, lo, hi)))
        self.on_slider_change()

    def _go_home(self):
        if self._estop:
            return
        if not self._homed:
            self.lbl_send.configure(text="⚠  Confirmez le homing avant d'envoyer une commande")
            return
        self._syncing_positions = True
        for i, value in enumerate(INIT_POS):
            self.pos_vars[i].set(value)
        self._syncing_positions = False
        self._update_3d()
        self.on_slider_change()
        self._log("Commande Home")

    def _save_preset(self, name):
        self._preset_slots[name] = [float(v.get()) for v in self.pos_vars]
        self._save_config()
        self._log(f"Preset {name} mémorisé")

    def _load_preset(self, name):
        if self._estop:
            return
        vals = self._preset_slots.get(name, INIT_POS)
        self._syncing_positions = True
        for i in range(4):
            self.pos_vars[i].set(float(vals[i]))
        self._syncing_positions = False
        self._update_3d()
        self.on_slider_change()
        self._log(f"Preset {name} rappelé")

    def _log(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] [{level}] {message}"
        self._log_lines.append(line)
        self._log_lines = self._log_lines[-200:]
        print(line, flush=True)
        if hasattr(self, "_log_box"):
            self._log_box.configure(state="normal")
            self._log_box.delete("1.0", "end")
            self._log_box.insert("end", "\n".join(self._log_lines) + "\n")
            self._log_box.see("end")
            self._log_box.configure(state="disabled")

    def _load_config(self):
        if not os.path.exists(CFG_PATH):
            return
        try:
            with open(CFG_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.ip = str(cfg.get("ip", self.ip))
            self._vs_cam_idx.set(str(cfg.get("vs_cam_idx", self._vs_cam_idx.get())))
            self._vs_method.set(str(cfg.get("vs_method", self._vs_method.get())))
            self._vs_marker_sz.set(str(cfg.get("vs_marker_sz", self._vs_marker_sz.get())))
            self._vs_work_z.set(str(cfg.get("vs_work_z", self._vs_work_z.get())))
            self._vs_work_z_robot.set(str(cfg.get("vs_work_z_robot", self._vs_work_z_robot.get())))
            self._vs_yolo_mdl.set(str(cfg.get("vs_yolo_mdl", self._vs_yolo_mdl.get())))
            self._vs_ft_x.set(str(cfg.get("vs_ft_x", self._vs_ft_x.get())))
            self._vs_ft_y.set(str(cfg.get("vs_ft_y", self._vs_ft_y.get())))
            self._vs_ft_z.set(str(cfg.get("vs_ft_z", self._vs_ft_z.get())))
            self._vs_gain.set(float(cfg.get("vs_gain", self._vs_gain.get())))
            self._vs_adaptive.set(bool(cfg.get("vs_adaptive", self._vs_adaptive.get())))
            self._vs_speed_ratio.set(float(cfg.get("vs_speed_ratio", self._vs_speed_ratio.get())))
            _ms = cfg.get("vs_max_steps", None)
            if isinstance(_ms, list) and len(_ms) == 4:
                for _i, _v in enumerate(_ms):
                    self._vs_max_step_vars[_i].set(str(_v))

            presets = cfg.get("presets", {})
            for key in ("P1", "P2", "P3"):
                vals = presets.get(key)
                if isinstance(vals, list) and len(vals) == 4:
                    self._preset_slots[key] = [float(v) for v in vals]
            self._ip_var.set(self.ip)
        except Exception:
            pass

    def _save_config(self):
        cfg = {
            "ip": self.ip,
            "vs_cam_idx": self._vs_cam_idx.get(),
            "vs_method": self._vs_method.get(),
            "vs_marker_sz": self._vs_marker_sz.get(),
            "vs_work_z": self._vs_work_z.get(),
            "vs_work_z_robot": self._vs_work_z_robot.get(),
            "vs_yolo_mdl": self._vs_yolo_mdl.get(),
            "vs_ft_x": self._vs_ft_x.get(),
            "vs_ft_y": self._vs_ft_y.get(),
            "vs_ft_z": self._vs_ft_z.get(),
            "vs_gain": float(self._vs_gain.get()),
            "vs_adaptive": bool(self._vs_adaptive.get()),
            "vs_speed_ratio": float(self._vs_speed_ratio.get()),
            "vs_max_steps": [v.get() for v in self._vs_max_step_vars],
            "presets": self._preset_slots,
        }
        try:
            with open(CFG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
        except Exception:
            pass

    def _on_close(self):
        try:
            self._save_config()
            self._stop_plc_polling()
            if self._reconnect_id is not None:
                self.root.after_cancel(self._reconnect_id)
                self._reconnect_id = None
            if self._vs_running:
                self._stop_vs()
            if self.connected:
                self.disconnect_plc()
        finally:
            self.root.destroy()

    # ═══════════════════════════════════════════════════════════════════════════
    # Coordonnées robot
    # ═══════════════════════════════════════════════════════════════════════════

    def _update_3d(self):
        enc = [v.get() for v in self.pos_vars]
        t1, t2, z_vis, t4 = enc_to_physical(enc)
        x1 = L1 * math.cos(t1);  y1 = L1 * math.sin(t1)
        x2 = x1 + L2*math.cos(t1+t2);  y2 = y1 + L2*math.sin(t1+t2)
        wrist = t1 + t2 + t4
        if hasattr(self, 'coord_labels'):
            self.coord_labels[0].configure(text=f"{x2:.1f}")
            self.coord_labels[1].configure(text=f"{y2:.1f}")
            self.coord_labels[2].configure(text=f"{z_vis:.1f}")
            self.coord_labels[3].configure(text=f"{math.degrees(wrist):.1f}")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = SCARAControlPanel()
    app.run()
