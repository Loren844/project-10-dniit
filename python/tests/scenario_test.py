import snap7
from snap7.util import set_dint, set_bool
import time

# --- TES RATIOS TROUVÉS AU TEST 1 ---
RATIO_DEG = 100.0
RATIO_MM  = 50.0

# Coordonnées (Theta1_deg, Z_mm, Theta3_deg)
POINTS = {
    "HOME":      (0.0,   0.0,   0.0),
    "APPROACH":  (45.0,  50.0,  -45.0),
    "GRAB":      (45.0,  150.0, -45.0), # Descend sur l'objet
    "LIFT":      (45.0,  20.0,  -45.0), # Remonte
    "PLACE":     (-45.0, 150.0, 45.0)   # Va déposer de l'autre côté
}

def move_robot(plc, nom_point, coords):
    print(f"\n>>> DÉPLACEMENT VERS : {nom_point} {coords}")
    
    t1_p = int(coords[0] * RATIO_DEG)
    z_p  = int(coords[1] * RATIO_MM)
    t3_p = int(coords[2] * RATIO_DEG)
    
    # 1. Écriture des coordonnées
    data = bytearray(12)
    set_dint(data, 0, t1_p)
    set_dint(data, 4, z_p)
    set_dint(data, 8, t3_p)
    plc.write_area(0x83, 0, 16, data)
    
    # 2. Trigger ON
    m2 = plc.read_area(0x83, 0, 2, 1)
    set_bool(m2, 0, 4, True)
    set_bool(m2, 0, 6, True)
    plc.write_area(0x83, 0, 2, m2)
    time.sleep(0.5)
    
    # 3. Trigger OFF
    set_bool(m2, 0, 4, False)
    set_bool(m2, 0, 6, False)
    plc.write_area(0x83, 0, 2, m2)
    
    print(f"    -> Ordre envoyé. Attente de la fin du mouvement...")

def set_gripper(plc, etat):
    m34 = plc.read_area(0x83, 0, 34, 1)
    set_bool(m34, 0, 5, etat)
    plc.write_area(0x83, 0, 34, m34)
    etat_str = "FERMÉE" if etat else "OUVERTE"
    print(f"*** PINCE {etat_str} ***")

try:
    print("========== SÉQUENCE PICK & PLACE (HARDCODED) ==========")
    plc = snap7.client.Client()
    plc.connect('192.168.0.10', 0, 1)
    
    # Assure-toi d'avoir fait le Homing physique avant de lancer ce script !
    
    move_robot(plc, "HOME", POINTS["HOME"])
    time.sleep(3) # Temps de trajet artificiel
    
    move_robot(plc, "APPROACH", POINTS["APPROACH"])
    time.sleep(3)
    
    move_robot(plc, "GRAB", POINTS["GRAB"])
    time.sleep(2)
    
    set_gripper(plc, True) # Ferme la pince
    time.sleep(1)
    
    move_robot(plc, "LIFT", POINTS["LIFT"])
    time.sleep(2)
    
    move_robot(plc, "PLACE", POINTS["PLACE"])
    time.sleep(4)
    
    set_gripper(plc, False) # Lâche l'objet
    time.sleep(1)
    
    move_robot(plc, "HOME", POINTS["HOME"])
    print("\n========== SÉQUENCE TERMINÉE AVEC SUCCÈS ==========")

except Exception as e:
    print(f"ERREUR : {e}")
finally:
    plc.disconnect()