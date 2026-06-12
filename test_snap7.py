from snap7.client import Client
import struct

print("Attempting connection from Ubuntu (192.168.0.10)...")
plc = Client()

try:
    plc.connect('192.168.0.10', 0, 1)
    print("Connection successful!")
    data = bytearray(12)
    struct.pack_into('>f', data, 0, 0.999)
    struct.pack_into('>f', data, 4, -0.555)
    struct.pack_into('>f', data, 8, 0.222)
    plc.db_write(1, 0, data)
    print("Write completed successfully!")
except Exception as e:
    print(f"Error: {e}")
finally:
    plc.disconnect()