from snap7.client import Client
import struct

print("Tentative de connexion depuis Ubuntu (192.168.0.10)...")
plc = Client()

try:
    plc.connect('192.168.0.10', 0, 1)
    print("Connexion réussie !")
    data = bytearray(12)
    struct.pack_into('>f', data, 0, 0.999)
    struct.pack_into('>f', data, 4, -0.555)
    struct.pack_into('>f', data, 8, 0.222)
    plc.db_write(1, 0, data)
    print("Écriture terminée avec succès !")
except Exception as e:
    print(f"Erreur : {e}")
finally:
    plc.disconnect()