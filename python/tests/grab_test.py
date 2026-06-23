import snap7
from snap7.util import set_bool
import time

print("========== TEST DE LA PINCE (M34.5) ==========")

try:
    plc = snap7.client.Client()
    plc.connect('192.168.0.10', 0, 1)
    
    print("[1] Fermeture de la pince dans 2 secondes...")
    time.sleep(2)
    m34_byte = plc.read_area(0x83, 0, 34, 1)
    set_bool(m34_byte, 0, 5, True) # Active le KEP
    plc.write_area(0x83, 0, 34, m34_byte)
    print("    -> Pince FERMÉE ! (Vérifie le robot)")
    
    print("[2] Maintien pendant 3 secondes...")
    time.sleep(3)
    
    print("[3] Ouverture de la pince...")
    set_bool(m34_byte, 0, 5, False) # Désactive le KEP
    plc.write_area(0x83, 0, 34, m34_byte)
    print("    -> Pince OUVERTE ! (Vérifie le robot)")

    print("========== TEST TERMINÉ ==========")

except Exception as e:
    print(f"ERREUR : {e}")
finally:
    plc.disconnect()