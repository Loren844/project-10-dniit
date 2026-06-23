import snap7
from snap7.util import set_dint, set_bool
import time

# --- PARAMÈTRES À TESTER ---
AXE_A_TESTER = 1      # Choisir : 1 (Theta1), 2 (Z), ou 3 (Theta3)
CIBLE_DEG_MM = 90.0   # Mettre 90.0 pour un angle, ou 100.0 pour l'axe Z (en mm)
RATIO        = 100.0  # LE FAMEUX RATIO À AJUSTER

def main():
    plc = snap7.client.Client()
    connected = False
    print(f"========== TEST DE L'AXE {AXE_A_TESTER} ==========")
    print(f"[1] Calcul en cours : Cible = {CIBLE_DEG_MM} (Ratio = {RATIO})")

    # Calcul des impulsions (pulses)
    impulsions_cibles = int(CIBLE_DEG_MM * RATIO)
    print(f"[2] Valeur convertie en impulsions : {impulsions_cibles}")
    
    try:
        print("[3] Connexion à l'automate 192.168.0.10...")
        plc = snap7.client.Client()
        plc.connect('192.168.0.10', 0, 1)
        print("    -> Connexion OK !")
        
        # Préparation du bloc mémoire (12 octets remplis de 0)
        data_coords = bytearray(12)
        
        # Écriture uniquement dans l'axe sélectionné
        if AXE_A_TESTER == 1:
            set_dint(data_coords, 0, impulsions_cibles)
            print(f"[4] Injection mémoire : Axe 1 (Offset 16) = {impulsions_cibles}")
        elif AXE_A_TESTER == 2:
            set_dint(data_coords, 4, impulsions_cibles)
            print(f"[4] Injection mémoire : Axe 2 (Offset 20) = {impulsions_cibles}")
        elif AXE_A_TESTER == 3:
            set_dint(data_coords, 8, impulsions_cibles)
            print(f"[4] Injection mémoire : Axe 3 (Offset 24) = {impulsions_cibles}")
            
        plc.write_area(0x83, 0, 16, data_coords)
        
        print("[5] Déclenchement du tir (Bits M2.4 & M2.6 sur ON)...")
        m2_byte = plc.read_area(0x83, 0, 2, 1)
        set_bool(m2_byte, 0, 4, True)
        set_bool(m2_byte, 0, 6, True)
        plc.write_area(0x83, 0, 2, m2_byte)
        
        time.sleep(1) # Laisse le temps au front montant d'être lu
        
        print("[6] Réarmement (Bits M2.4 & M2.6 sur OFF)...")
        set_bool(m2_byte, 0, 4, False)
        set_bool(m2_byte, 0, 6, False)
        plc.write_area(0x83, 0, 2, m2_byte)

        print("========== TEST TERMINÉ ==========")
        print("-> Mesure physiquement le robot maintenant.")
        
    except Exception as e:
        print(f"Erreur durant le test : {e}")
        
    finally:
        # CE BLOC EST LA CLÉ DE LA STABILITÉ
        if connected:
            # On remet les bits de commande à FALSE par sécurité
            try:
                m2 = plc.read_area(0x83, 0, 2, 1)
                set_bool(m2, 0, 4, False)
                set_bool(m2, 0, 6, False)
                plc.write_area(0x83, 0, 2, m2)
                
                plc.disconnect()
                print("Déconnexion propre effectuée.")
            except:
                print("Erreur lors de la déconnexion.")

if __name__ == "__main__":
    main()





