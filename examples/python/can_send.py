import can
import struct
import time


can0 = can.interface.Bus(channel = 'can0', interface = 'socketcan', fd=True)
msg = 2.5
while(1):
     t = time.time()
     data = bytearray(struct.pack("f", msg))
     msg_to_send = can.Message(arbitration_id=0x065, data = data)
     can0.send(msg_to_send)
     time.sleep(0.001) 