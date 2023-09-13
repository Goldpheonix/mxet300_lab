import L2_compass_heading
import time
import L1_ina
import L1_log
while(1):
    reading = L2_compass_heading.get_heading()
    print(reading)
    if reading > 125.00 and reading < 170.00:
     cardinaldirection = "North"
     print(cardinaldirection)
    elif reading > 40.00 and reading < 80.00:
     cardinaldirection = "East"
     print(cardinaldirection)
    elif reading > 80.00 and reading < 125.00:
     cardinaldirection = "North-East"
     print(cardinaldirection)
    elif reading < -40.00 and reading > -80.00:
      cardinaldirection = "West"
      print(cardinaldirection)
    elif reading > -20.00 and reading < 20.00:
      cardinaldirection = "South"
      print(cardinaldirection)
    elif reading > -40.00 and reading < -20.00:
      cardinaldirection = "South-West"
      print(cardinaldirection)
    elif reading < -80.00 and reading > -175.00:
      cardinaldirection = "North-West"
      print(cardinaldirection)
    elif reading > 20.00 and reading < 40.00:
      cardinaldirection = "South-East"
      print(cardinaldirection)
    voltageread = L1_ina.readVolts()
    L1_log.uniqueFile(voltageread, "voltage")
    L1_log.uniqueFile(reading, "compassheading")
    L1_log.stringTmpFile(cardinaldirection, "cardinaldirection")
    time.sleep(.1)