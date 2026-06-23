import snap7
from snap7.util import set_dint, set_bool
import time

plc = snap7.client.Client()
plc.connect('192.168.0.10', 0, 1)

data = bytearray(12)
set_dint(data, 0, 1000)
set_dint(data, 4, 5000)
set_dint(data, 8, 2000)
plc.write_area(0x83, 0, 16, data)

m2_byte = plc.read_area(0x83, 0, 2, 1)
set_bool(m2_byte, 0, 4, True)
set_bool(m2_byte, 0, 6, True)
plc.write_area(0x83, 0, 2, m2_byte)

time.sleep(1)

set_bool(m2_byte, 0, 4, False)
set_bool(m2_byte, 0, 6, False)
plc.write_area(0x83, 0, 2, m2_byte)

plc.disconnect()