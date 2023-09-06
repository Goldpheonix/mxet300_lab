import L1_ina
import L1_log
import time
while(1):
    voltageread = L1_ina.readVolts()
    L1_log.uniqueFile(voltageread, "voltage")
    time.sleep(.1)
