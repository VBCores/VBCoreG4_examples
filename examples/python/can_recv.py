import can
import struct
import time


can0 = can.interface.Bus(channel = 'can0', interface = 'socketcan', fd=True)
while(1):
     t = time.time()
     msg = can0.recv()
     if msg.arbitration_id == 101:
          data = struct.unpack('f', msg.data)
          print(data)
     time.sleep(0.001) 