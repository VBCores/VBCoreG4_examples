import can
import struct
import time


can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan', fd=True)
while(1):
     t = time.time()
     msg = can0.recv()
     if msg.arbitration_id == 101:
          data = struct.unpack('f', msg.data)
          print(data)
     time.sleep(0.002 - (time.time()-t)) # время работы тела цикла - 2 мс