import snap7
from snap7.util import set_real, get_real, set_bool, get_bool
import time
import sys

IP_PLC = '192.168.0.10'
PULSE_DURATION = 0.5

def print_header(title):
    print(f"\n{'='*50}")
    print(f" {title.upper()}")
    print(f"{'='*50}")

def diagnostic_complet(plc):
    print_header("SCAN DU CERVEAU DE L'AUTOMATE")
    try:
        inputs = plc.read_area(0x81, 0, 0, 2)
        
        merkers_base = plc.read_area(0x83, 0, 0, 40)
        
        merkers_cibles = plc.read_area(0x83, 0, 100, 32)
        
        print("\n--- BOUTONS ET SWITCHS PHYSIQUES ---")
        print(f"[I1.4] Switch START      : {'ON (Autorisé)' if get_bool(inputs, 1, 4) else 'OFF (Bloqué)'}")
        print(f"[I0.7] Mode AUTO         : {'ON' if get_bool(inputs, 0, 7) else 'OFF'}")
        print(f"[I0.6] Mode MANUEL       : {'ON' if get_bool(inputs, 0, 6) else 'OFF'}")
        print(f"[I1.5] Bouton HOME       : {'Appuyé' if get_bool(inputs, 1, 5) else 'Relâché'}")
        
        print("\n--- CAPTEURS FINS DE COURSE (CTHT) ---")
        for bit, num in enumerate(range(1, 5)):
            print(f"[I1.{bit}] CTHT {num}           : {'DÉTECTÉ' if get_bool(inputs, 1, bit) else 'Rien'}")

        print("\n--- MÉMOIRE DES CIBLES ACTUELLES (REAL) ---")
        val_a1 = get_real(merkers_cibles, 0)
        val_a2 = get_real(merkers_cibles, 4)
        val_a3 = get_real(merkers_cibles, 8)
        val_a4 = get_real(merkers_cibles, 12)
        print(f"[MD100] Cible Axe 1       : {val_a1:.2f}")
        print(f"[MD104] Cible Axe 2       : {val_a2:.2f}")
        print(f"[MD108] Cible Axe 3       : {val_a3:.2f}")
        print(f"[MD112] Cible Axe 4       : {val_a4:.2f}")
        
        print("\n--- ÉTAT DE LA PINCE ---")
        print(f"[M34.5] KEP (Pince)      : {'FERMÉE' if get_bool(merkers_base, 34, 5) else 'OUVERTE'}")

    except Exception as e:
        print(f"[ERREUR LECTURE] Impossible de scanner l'automate : {e}")

def tester_axe(plc, numero_axe, offset_pos, offset_vit, valeur_cible):
    print_header(f"TEST DE L'AXE {numero_axe}")
    try:
        print(f"1. Préparation de la valeur : {valeur_cible}")
        data_pos = bytearray(4)
        data_vit = bytearray(4)
        set_real(data_pos, 0, valeur_cible)
        set_real(data_vit, 0, 50.0)
        
        print(f"2. Injection mémoire Pos MD{offset_pos} et Vit MD{offset_vit}...")
        plc.write_area(0x83, 0, offset_pos, data_pos)
        plc.write_area(0x83, 0, offset_vit, data_vit)
        
        verif = plc.read_area(0x83, 0, offset_pos, 4)
        val_lue = get_real(verif, 0)
        print(f"   -> Vérification mémoire PLC : {val_lue:.2f}")
        
        print("3. Activation du trigger (M2.6) -> ON")
        m2 = plc.read_area(0x83, 0, 2, 1)
        set_bool(m2, 0, 6, True)
        plc.write_area(0x83, 0, 2, m2)
        
        print(f"4. Maintien du signal pendant {PULSE_DURATION} secondes...")
        time.sleep(PULSE_DURATION)
        
        print("5. Relâchement du trigger -> OFF")
        set_bool(m2, 0, 6, False)
        plc.write_area(0x83, 0, 2, m2)
        
        print(">>> COMMANDE ENVOYÉE AVEC SUCCÈS. Regarde le robot !")

    except Exception as e:
        print(f"[ERREUR ÉCRITURE] Le test a échoué : {e}")

def tester_pince(plc):
    print_header("TEST DE LA PINCE (M34.5)")
    try:
        m34 = plc.read_area(0x83, 0, 34, 1)
        etat_actuel = get_bool(m34, 0, 5)
        
        nouvel_etat = not etat_actuel
        action = "FERMER" if nouvel_etat else "OUVRIR"
        
        print(f"La pince est actuellement {'FERMÉE' if etat_actuel else 'OUVERTE'}.")
        print(f"Envoi de l'ordre pour {action} la pince...")
        
        set_bool(m34, 0, 5, nouvel_etat)
        plc.write_area(0x83, 0, 34, m34)
        print(">>> ORDRE ENVOYÉ.")
        
    except Exception as e:
        print(f"[ERREUR ÉCRITURE] Impossible de bouger la pince : {e}")

def main():
    plc = snap7.client.Client()
    connected = False
    
    try:
        print(f"Tentative de connexion à l'automate ({IP_PLC})...")
        plc.connect(IP_PLC, 0, 1)
        connected = True
        print(">>> CONNEXION ÉTABLIE ! <<<\n")
        
        while True:
            print_header("MENU DE DIAGNOSTIC V2 (REAL)")
            print("1. Lancer un Scan du cerveau de l'automate (LIRE)")
            print("2. Tester la PINCE (Ouverture / Fermeture)")
            print("3. Tester l'AXE 1 (Rotation base - MD100)")
            print("4. Tester l'AXE 2 (Vertical Z - MD104)")
            print("5. Tester l'AXE 3 (Rotation coude - MD108)")
            print("6. Tester l'AXE 4 (Rotation Pince - MD112)")
            print("0. Quitter proprement")
            
            choix = input("\nChoisis une option (0-6) : ")
            
            if choix == '1':
                diagnostic_complet(plc)
            elif choix == '2':
                tester_pince(plc)
            elif choix == '3':
                val = float(input("Valeur (Real) pour l'Axe 1 ? : "))
                tester_axe(plc, 1, 100, 116, val)
            elif choix == '4':
                val = float(input("Valeur (Real) pour l'Axe 2 ? : "))
                tester_axe(plc, 2, 104, 120, val)
            elif choix == '5':
                val = float(input("Valeur (Real) pour l'Axe 3 ? : "))
                tester_axe(plc, 3, 108, 124, val)
            elif choix == '6':
                val = float(input("Valeur (Real) pour l'Axe 4 ? : "))
                tester_axe(plc, 4, 112, 128, val)
            elif choix == '0':
                print("Fermeture demandée...")
                break
            else:
                print("Choix invalide.")
            
            input("\nAppuie sur ENTRÉE pour continuer...")
            
    except Exception as e:
        print(f"\n[ERREUR CRITIQUE] Le programme a crashé : {e}")
        
    finally:
        if connected:
            print("\nNettoyage et déconnexion...")
            try:
                m2 = plc.read_area(0x83, 0, 2, 1)
                set_bool(m2, 0, 6, False)
                plc.write_area(0x83, 0, 2, m2)
                
                plc.disconnect()
                print(">>> Déconnexion PROPRE effectuée.")
            except:
                print("Échec du nettoyage final.")

if __name__ == "__main__":
    main()