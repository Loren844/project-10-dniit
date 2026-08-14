"""
gamepad_robot.py — Contrôle du robot SCARA via manette de jeu
==============================================================
Dépendances :  pip install pygame snap7

Mapping manette (Xbox / PS4 / générique) :
  Stick gauche  X  →  J1 (épaule)
  Stick gauche  Y  →  J2 (coude)
  Stick droit   Y  →  Z  (hauteur)
  Stick droit   X  →  J4 (orientation)
  LB / RB          →  vitesse  −/+
  A  (×)           →  Ouvrir/Fermer pince
  B  (○)           →  E-STOP toggle
  Start (Options)  →  Retour Home
  Back  (Share)    →  Quitter
"""

import sys
import time
import math
import json
import os
import threading
import argparse

try:
    import pygame
except ImportError:
    sys.exit("pygame non installé — lancez :  pip install pygame")

try:
    import snap7
    from snap7.util import set_real, set_bool, get_bool, get_real
except ImportError:
    sys.exit("snap7 non installé — lancez :  pip install python-snap7")

# ── Paramètres robot (identiques à gui_robot.py) ─────────────────────────────
LIMITS      = [(-16000., 20000.), (-11000., 11000.),
               (-6000.,   9500.), (-2000.,   2300.)]
INIT_POS    = [0.0, 0.0, 0.0, 0.0]
INIT_SPEEDS = [22000., 25000., 22000., 1800.]

# Ordre attendu par la PLC : J1, Z, J2, J4
PLC_AXIS_ORDER    = [0, 2, 1, 3]
PLC_TO_UI_ORDER   = [0, 2, 1, 3]

AXIS_NAMES  = ["J1 — Épaule", "J2 — Coude", "Z — Hauteur", "J4 — Orient."]
AXIS_COLORS = [(233, 69, 96), (255, 140, 66), (0, 212, 255), (168, 85, 247)]

# Vitesse de jog en unités encodeur par seconde (à 100 % du stick)
JOG_SPEED   = [8000., 8000., 4000., 800.]
# Pas de temps de la boucle de contrôle (s)
LOOP_DT     = 0.05   # 20 Hz

# Dead-zone stick (évite la dérive)
DEAD_ZONE   = 0.08

# Niveaux de vitesse globale (facteur multiplicatif)
SPEED_LEVELS     = [0.10, 0.25, 0.50, 0.75, 1.00]
DEFAULT_SPD_IDX  = 2  # 50 %

# ── Mapping boutons (Xbox USB / D-Input générique) ────────────────────────────
# Modifiez ces indices si votre manette diffère (consultez gamepad_robot.py -m)
BTN_GRIPPER = 0   # A / ×
BTN_ESTOP   = 1   # B / ○
BTN_HOME    = 7   # Start / Options
BTN_QUIT    = 6   # Back / Share
BTN_SPD_DN  = 4   # LB / L1
BTN_SPD_UP  = 5   # RB / R1

# Axes stick  (Linux/Windows standard)
AX_J1  = 0   # stick gauche X
AX_J2  = 1   # stick gauche Y  (inversé → +haut = +J2)
AX_Z   = 4   # stick droit  Y  (inversé)
AX_J4  = 3   # stick droit  X

CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'gui_robot_config.json')


# ─────────────────────────────────────────────────────────────────────────────
def load_plc_ip() -> str:
    """Lit l'IP de la PLC depuis gui_robot_config.json si disponible."""
    try:
        with open(CFG_PATH) as f:
            return json.load(f).get("ip", "192.168.0.10")
    except Exception:
        return "192.168.0.10"


def apply_dead_zone(value: float, dz: float = DEAD_ZONE) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


# ─────────────────────────────────────────────────────────────────────────────
class RobotGamepad:
    """Boucle principale : lit la manette et envoie les commandes à la PLC."""

    def __init__(self, plc_ip: str, rack: int = 0, slot: int = 1):
        self.plc_ip   = plc_ip
        self.rack     = rack
        self.slot     = slot

        self.plc      = snap7.client.Client()
        self._lock    = threading.Lock()
        self.connected = False

        self.positions  = list(INIT_POS)       # ordre UI [J1, J2, Z, J4]
        self.speeds     = list(INIT_SPEEDS)
        self.spd_idx    = DEFAULT_SPD_IDX
        self.estop      = False
        self.pince_open = True
        self.running    = True

        # Historique des messages d'état (affiché dans l'overlay pygame)
        self._log_lines: list[str] = []

    # ── PLC helpers ──────────────────────────────────────────────────────────
    def connect(self):
        try:
            self.plc.connect(self.plc_ip, self.rack, self.slot)
            self.connected = True
            self._log(f"✓ PLC connectée ({self.plc_ip})")
            self._sync_from_plc()
        except Exception as e:
            self.connected = False
            self._log(f"✗ Connexion PLC impossible : {e}")

    def disconnect(self):
        try:
            self.plc.disconnect()
        except Exception:
            pass
        self.connected = False

    def _plc_read(self, start: int, size: int) -> bytearray:
        with self._lock:
            return self.plc.read_area(0x83, 0, start, size)

    def _plc_write(self, start: int, data: bytearray):
        with self._lock:
            self.plc.write_area(0x83, 0, start, data)

    def _sync_from_plc(self):
        """Lit les positions actuelles depuis la PLC pour initialiser l'état."""
        try:
            raw = self._plc_read(100, 16)
            plc_pos = [get_real(raw, i * 4) for i in range(4)]
            # PLC order → UI order
            self.positions = [plc_pos[i] for i in PLC_TO_UI_ORDER]
            self._log("Positions synchronisées depuis la PLC")
        except Exception as e:
            self._log(f"Sync PLC impossible : {e}")

    def _send_positions(self):
        """Envoie les positions + vitesses à la PLC et déclenche le mouvement."""
        if not self.connected or self.estop:
            return
        try:
            data = bytearray(32)
            plc_pos = [self.positions[i] for i in PLC_AXIS_ORDER]
            plc_vit = [self.speeds[i]    for i in PLC_AXIS_ORDER]
            for i in range(4):
                set_real(data, i * 4,      plc_pos[i])
                set_real(data, 16 + i * 4, plc_vit[i])
            self._plc_write(100, data)

            # Impulsion de déclenchement sur M50.0
            m50 = self._plc_read(50, 1)
            set_bool(m50, 0, 0, True)
            self._plc_write(50, m50)
            time.sleep(0.05)
            set_bool(m50, 0, 0, False)
            self._plc_write(50, m50)
        except Exception as e:
            self._log(f"Erreur envoi PLC : {e}")

    def _set_pince(self, open_: bool):
        if not self.connected:
            return
        try:
            m34 = self._plc_read(34, 1)
            set_bool(m34, 0, 5, not open_)   # bit 5 = pince fermée
            self._plc_write(34, m34)
            self.pince_open = open_
            self._log("Pince " + ("ouverte" if open_ else "fermée"))
        except Exception as e:
            self._log(f"Erreur pince : {e}")

    def _go_home(self):
        self.positions = list(INIT_POS)
        self._send_positions()
        self._log("→ Home")

    # ── Log overlay ──────────────────────────────────────────────────────────
    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        self._log_lines.append(line)
        if len(self._log_lines) > 8:
            self._log_lines.pop(0)

    # ── Boucle principale ────────────────────────────────────────────────────
    def run(self):
        pygame.init()
        pygame.joystick.init()

        # Fenêtre d'état (peut être minimisée)
        screen = pygame.display.set_mode((680, 480))
        pygame.display.set_caption("SCARA Gamepad Control")
        clock  = pygame.time.Clock()

        font_big  = pygame.font.SysFont("monospace", 18, bold=True)
        font_med  = pygame.font.SysFont("monospace", 14)
        font_sml  = pygame.font.SysFont("monospace", 11)

        if pygame.joystick.get_count() == 0:
            self._log("⚠  Aucune manette détectée — branchez-en une puis relancez.")
        joystick = None
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            self._log(f"Manette : {joystick.get_name()}")

        self.connect()

        # État boutons précédent (pour détecter front montant)
        prev_btn: dict[int, bool] = {}

        def btn_pressed(idx: int) -> bool:
            if joystick is None or idx >= joystick.get_numbuttons():
                return False
            cur = bool(joystick.get_button(idx))
            was = prev_btn.get(idx, False)
            prev_btn[idx] = cur
            return cur and not was

        send_needed = False

        while self.running:
            dt = clock.tick(int(1.0 / LOOP_DT * 1000) // 1000 or 20) / 1000.0

            # ── Événements pygame ─────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.JOYDEVICEADDED:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    self._log(f"Manette connectée : {joystick.get_name()}")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    joystick = None
                    self._log("⚠  Manette déconnectée")

            if joystick is None:
                self._draw(screen, font_big, font_med, font_sml, None)
                pygame.display.flip()
                continue

            # Mise à jour état précédent pour tous les boutons
            for i in range(joystick.get_numbuttons()):
                prev_btn.setdefault(i, False)

            # ── Boutons (front montant) ────────────────────────────────────
            if btn_pressed(BTN_QUIT):
                self.running = False

            if btn_pressed(BTN_ESTOP):
                self.estop = not self.estop
                self._log("E-STOP " + ("ACTIVÉ ⛔" if self.estop else "relâché ✓"))

            if not self.estop:
                if btn_pressed(BTN_GRIPPER):
                    self._set_pince(not self.pince_open)

                if btn_pressed(BTN_HOME):
                    self._go_home()

                if btn_pressed(BTN_SPD_UP) and self.spd_idx < len(SPEED_LEVELS) - 1:
                    self.spd_idx += 1
                    self._log(f"Vitesse : {int(SPEED_LEVELS[self.spd_idx]*100)} %")

                if btn_pressed(BTN_SPD_DN) and self.spd_idx > 0:
                    self.spd_idx -= 1
                    self._log(f"Vitesse : {int(SPEED_LEVELS[self.spd_idx]*100)} %")

                # ── Axes (jog continu) ─────────────────────────────────────
                speed_factor = SPEED_LEVELS[self.spd_idx]
                axes_raw = {
                    0: apply_dead_zone( joystick.get_axis(AX_J1)),   # J1
                    1: apply_dead_zone(-joystick.get_axis(AX_J2)),   # J2  (−Y)
                    2: apply_dead_zone(-joystick.get_axis(AX_Z)),    # Z   (−Y)
                    3: apply_dead_zone( joystick.get_axis(AX_J4)),   # J4
                }

                moved = False
                for ax, raw in axes_raw.items():
                    if raw != 0.0:
                        delta = raw * JOG_SPEED[ax] * speed_factor * dt
                        lo, hi = LIMITS[ax]
                        self.positions[ax] = float(
                            max(lo, min(hi, self.positions[ax] + delta))
                        )
                        moved = True

                if moved:
                    send_needed = True

            # Envoi PLC périodique (≤ 1 envoi par cycle de LOOP_DT)
            if send_needed and not self.estop:
                threading.Thread(target=self._send_positions, daemon=True).start()
                send_needed = False

            # ── Affichage ─────────────────────────────────────────────────
            self._draw(screen, font_big, font_med, font_sml, joystick)
            pygame.display.flip()

        # Nettoyage
        self.disconnect()
        pygame.quit()

    # ── Rendu de l'overlay ───────────────────────────────────────────────────
    def _draw(self, screen, font_big, font_med, font_sml, joystick):
        BG   = (13, 17, 23)
        LINE = (30, 40, 55)
        WHITE = (220, 220, 220)
        GRAY  = (120, 120, 140)
        RED   = (220, 60, 60)
        GREEN = (0, 200, 120)
        CYAN  = (0, 212, 255)

        screen.fill(BG)

        # ── Titre ─────────────────────────────────────────────────────────
        title = font_big.render("SCARA — Contrôle Manette", True, CYAN)
        screen.blit(title, (20, 14))

        # ── Statut connexion / E-STOP ─────────────────────────────────────
        plc_txt   = f"PLC  {self.plc_ip}  {'✓ connectée' if self.connected else '✗ hors ligne'}"
        plc_col   = GREEN if self.connected else RED
        screen.blit(font_med.render(plc_txt, True, plc_col), (20, 44))

        if self.estop:
            estop_surf = font_big.render("⛔  E-STOP ACTIF", True, RED)
            screen.blit(estop_surf, (20, 70))

        spd_txt = f"Vitesse  {int(SPEED_LEVELS[self.spd_idx]*100):3d} %   [LB−/RB+]"
        screen.blit(font_med.render(spd_txt, True, WHITE), (360, 44))

        # ── Barres de position ────────────────────────────────────────────
        y0 = 105
        for i, (name, color) in enumerate(zip(AXIS_NAMES, AXIS_COLORS)):
            lo, hi   = LIMITS[i]
            val      = self.positions[i]
            ratio    = (val - lo) / (hi - lo) if hi > lo else 0.5
            bar_w    = 280
            bar_h    = 16
            bx, by   = 20, y0 + i * 80

            pygame.draw.rect(screen, LINE,  (bx, by + 22, bar_w, bar_h), border_radius=4)
            pygame.draw.rect(screen, color, (bx, by + 22, int(bar_w * ratio), bar_h),
                             border_radius=4)
            pygame.draw.rect(screen, (60, 70, 85), (bx, by + 22, bar_w, bar_h), 1,
                             border_radius=4)

            lbl  = font_med.render(f"{name}", True, color)
            val_ = font_med.render(f"{val:+9.1f}  enc", True, WHITE)
            screen.blit(lbl,  (bx, by))
            screen.blit(val_, (bx + bar_w + 12, by + 22))

        # ── État pince ────────────────────────────────────────────────────
        pince_txt = "PINCE :  " + ("OUVERTE ⊙" if self.pince_open else "FERMÉE ⊗")
        pince_col = (255, 187, 51) if self.pince_open else (255, 136, 0)
        screen.blit(font_med.render(pince_txt, True, pince_col), (360, 105))

        # ── Manette : visu sticks ─────────────────────────────────────────
        if joystick is not None:
            cx1, cy = 420, 200
            cx2     = 560
            r       = 45
            for cx, label in ((cx1, "Gauche"), (cx2, "Droit")):
                pygame.draw.circle(screen, LINE,  (cx, cy), r, 1)
                screen.blit(font_sml.render(label, True, GRAY),
                            (cx - 20, cy + r + 4))

            raw_lx = apply_dead_zone( joystick.get_axis(AX_J1))
            raw_ly = apply_dead_zone(-joystick.get_axis(AX_J2))
            raw_rx = apply_dead_zone( joystick.get_axis(AX_J4))
            raw_ry = apply_dead_zone(-joystick.get_axis(AX_Z))

            def dot(cx, cy, nx, ny, col):
                px = int(cx + nx * r * 0.9)
                py = int(cy - ny * r * 0.9)
                pygame.draw.circle(screen, col, (px, py), 7)

            dot(cx1, cy, raw_lx, raw_ly, AXIS_COLORS[0])
            dot(cx2, cy, raw_rx, raw_ry, AXIS_COLORS[2])

        # ── Log messages ──────────────────────────────────────────────────
        log_y = 340
        screen.blit(font_sml.render("─── LOG ───", True, GRAY), (20, log_y - 14))
        for line in self._log_lines[-6:]:
            screen.blit(font_sml.render(line, True, GRAY), (20, log_y))
            log_y += 14

        # ── Aide rapide ───────────────────────────────────────────────────
        help_lines = [
            "Stick G X/Y → J1/J2   Stick D X/Y → J4/Z",
            "A=Pince  B=E-Stop  Start=Home  Back=Quitter  LB/RB=Vitesse",
        ]
        hy = 452
        for hl in help_lines:
            screen.blit(font_sml.render(hl, True, GRAY), (20, hy))
            hy += 13


# ── Mode diagnostic : liste les axes/boutons de la manette ───────────────────
def list_gamepad_inputs():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Aucune manette détectée.")
        pygame.quit()
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Manette : {js.get_name()}")
    print(f"  Axes    : {js.get_numaxes()}")
    print(f"  Boutons : {js.get_numbuttons()}")
    print(f"  Hats    : {js.get_numhats()}")
    print("\nAppuyez sur Ctrl+C pour quitter, ou bougez les axes / boutons :\n")
    clock = pygame.time.Clock()
    try:
        while True:
            pygame.event.pump()
            parts = [f"A{i}={js.get_axis(i):+.2f}" for i in range(js.get_numaxes())]
            parts += [f"B{i}={js.get_button(i)}" for i in range(js.get_numbuttons())]
            print("\r" + "  ".join(parts) + "   ", end="", flush=True)
            clock.tick(20)
    except KeyboardInterrupt:
        pass
    pygame.quit()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Contrôle du robot SCARA par manette de jeu")
    parser.add_argument("--ip",   default=None,
                        help="IP de la PLC (défaut : valeur de gui_robot_config.json)")
    parser.add_argument("--rack", type=int, default=0, help="Rack PLC (défaut 0)")
    parser.add_argument("--slot", type=int, default=1, help="Slot PLC (défaut 1)")
    parser.add_argument("-m", "--map", action="store_true",
                        help="Mode diagnostic : affiche les axes et boutons en temps réel")
    args = parser.parse_args()

    if args.map:
        list_gamepad_inputs()
    else:
        ip = args.ip or load_plc_ip()
        controller = RobotGamepad(plc_ip=ip, rack=args.rack, slot=args.slot)
        controller.run()
