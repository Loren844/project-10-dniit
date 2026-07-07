import tkinter as tk
from tkinter import ttk
import snap7
from snap7.util import set_real, set_bool, get_bool
import threading
import time

class ModernRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Control Panel Pro")
        self.root.geometry("700x550")
        self.root.configure(bg="#2b2b2b")

        self.plc = snap7.client.Client()
        self.ip = '192.168.0.10'
        self.connected = False
        self.pince_state = False
        self.send_timer = None
        self.is_sending = False

        self.pos_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.vit_vars = [tk.DoubleVar(value=v) for v in [22000.0, 25000.0, 22000.0, 1800.0]]
        
        self.limits = [
            (-25000.0, 25000.0),
            (-25000.0, 25000.0),
            (-25000.0, 25000.0),
            (-2000.0, 2000.0)
        ]

        self.setup_ui()

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background="#2b2b2b")
        style.configure("Horizontal.TScale", background="#3c3f41", troughcolor="#2b2b2b", sliderthickness=20)

        header_frame = tk.Frame(self.root, bg="#1e1e1e", pady=15)
        header_frame.pack(fill="x")

        self.lbl_status = tk.Label(header_frame, text="DÉCONNECTÉ", bg="#1e1e1e", fg="#ff4444", font=("Segoe UI", 16, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=30)

        tk.Button(header_frame, text="Connecter", command=self.connect_plc, bg="#00C851", fg="white", font=("Segoe UI", 11, "bold"), relief="flat", width=12).pack(side=tk.RIGHT, padx=10)
        tk.Button(header_frame, text="Déconnecter", command=self.disconnect_plc, bg="#ff4444", fg="white", font=("Segoe UI", 11, "bold"), relief="flat", width=12).pack(side=tk.RIGHT, padx=10)

        main_frame = tk.Frame(self.root, bg="#2b2b2b")
        main_frame.pack(pady=20, padx=30, fill="both", expand=True)

        for i in range(4):
            frame = tk.Frame(main_frame, bg="#3c3f41", bd=0, highlightbackground="#555555", highlightthickness=1)
            frame.pack(fill="x", pady=8, ipadx=10, ipady=15)
            
            lbl = tk.Label(frame, text=f"AXE {i+1}", bg="#3c3f41", fg="#00d2ff", font=("Segoe UI", 14, "bold"), width=8)
            lbl.pack(side=tk.LEFT, padx=10)
            
            val_lbl = tk.Label(frame, textvariable=self.pos_vars[i], bg="#1e1e1e", fg="white", font=("Consolas", 12, "bold"), width=10)
            val_lbl.pack(side=tk.LEFT, padx=10)

            scale = ttk.Scale(frame, from_=self.limits[i][0], to=self.limits[i][1], variable=self.pos_vars[i], orient="horizontal", command=self.on_slider_change)
            scale.pack(side=tk.LEFT, fill="x", expand=True, padx=20)

        self.btn_pince = tk.Button(self.root, text="PINCE : OUVERTE", command=self.toggle_pince, bg="#ffbb33", fg="black", font=("Segoe UI", 14, "bold"), relief="flat", width=25, pady=12)
        self.btn_pince.pack(pady=10)

    def connect_plc(self):
        try:
            self.plc.connect(self.ip, 0, 1)
            self.connected = True
            self.lbl_status.config(text="CONNECTÉ", fg="#00C851")
            self.read_pince()
        except:
            self.lbl_status.config(text="ERREUR", fg="#ff4444")

    def disconnect_plc(self):
        if self.connected:
            self.plc.disconnect()
            self.connected = False
            self.lbl_status.config(text="DÉCONNECTÉ", fg="#ff4444")

    def read_pince(self):
        try:
            m34 = self.plc.read_area(0x83, 0, 34, 1)
            self.pince_state = get_bool(m34, 0, 5)
            self.update_btn()
        except:
            pass

    def update_btn(self):
        if self.pince_state:
            self.btn_pince.config(text="PINCE : FERMÉE", bg="#ff8800", fg="white")
        else:
            self.btn_pince.config(text="PINCE : OUVERTE", bg="#ffbb33", fg="black")

    def toggle_pince(self):
        if not self.connected: return
        try:
            self.pince_state = not self.pince_state
            m34 = self.plc.read_area(0x83, 0, 34, 1)
            set_bool(m34, 0, 5, self.pince_state)
            self.plc.write_area(0x83, 0, 34, m34)
            self.update_btn()
        except:
            pass

    def on_slider_change(self, event=None):
        if not self.connected: return
        
        for i in range(4):
            self.pos_vars[i].set(round(self.pos_vars[i].get(), 1))

        if self.send_timer is not None:
            self.root.after_cancel(self.send_timer)
        self.send_timer = self.root.after(200, self.trigger_move)

    def trigger_move(self):
        if not self.is_sending:
            threading.Thread(target=self.send_to_plc, daemon=True).start()

    def send_to_plc(self):
        self.is_sending = True
        try:
            data = bytearray(32)
            for i in range(4):
                set_real(data, i*4, float(self.pos_vars[i].get()))
                set_real(data, 16 + i*4, float(self.vit_vars[i].get()))
            
            self.plc.write_area(0x83, 0, 100, data)
            
            m50 = self.plc.read_area(0x83, 0, 50, 1)
            set_bool(m50, 0, 0, True)
            self.plc.write_area(0x83, 0, 50, m50)
            
            time.sleep(0.3)
            
            set_bool(m50, 0, 0, False)
            self.plc.write_area(0x83, 0, 50, m50)
        except:
            pass
        finally:
            self.is_sending = False

if __name__ == "__main__":
    root = tk.Tk()
    app = ModernRobotGUI(root)
    root.mainloop()
