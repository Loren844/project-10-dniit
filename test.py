import snap7
from snap7.util import set_real, set_bool
import time

IP_PLC = '192.168.0.10'

def main():
    plc = snap7.client.Client()
    try:
        plc.connect(IP_PLC, 0, 1)
        
        data = bytearray(32)
        set_real(data, 0, -2000.0)
        set_real(data, 4, 2000.0)
        set_real(data, 8, -3000.0)
        set_real(data, 12, 1000.0)
        
        set_real(data, 16, 22000.0)
        set_real(data, 20, 25000.0)
        set_real(data, 24, 22000.0)
        set_real(data, 28, 2000.0)
        
        plc.write_area(0x83, 0, 100, data)
        
        m50 = plc.read_area(0x83, 0, 50, 1)
        set_bool(m50, 0, 0, True)
        plc.write_area(0x83, 0, 50, m50)
        
        time.sleep(0.5)
        
        set_bool(m50, 0, 0, False)
        plc.write_area(0x83, 0, 50, m50)
        
        print(">>> NOUVELLES COORDONNEES ENVOYEES ET TRIGGER REINITIALISE !")
        
    except Exception as e:
        print(e)
    finally:
        plc.disconnect()

if __name__ == "__main__":
    main()
