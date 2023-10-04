import L1_log
import L2_vector
import L1_lidar
import L1_ina
import time
while(1):
    distance = L2_vector.getNearest()
    voltageread = L1_ina.readVolts()
    print(distance[0])
    print(distance[1])
    L1_log.uniqueFile(distance[0],"DistanceLI")
    L1_log.uniqueFile(distance[1],"AngleLI")
    L1_log.uniqueFile(voltageread,"voltage")
    cartesian = L2_vector.polar2cart(distance[0], distance[1])
    L1_log.uniqueFile(cartesian[0], "XCart")
    L1_log.uniqueFile(cartesian[1],"YCart")
    time.sleep(.1)