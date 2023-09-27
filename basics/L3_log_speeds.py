import L2_kinematics
import time
import L1_ina
import L1_log
import L1_encoder
import numpy

while(1):
    xdot= L2_kinematics.getMotion()
    phis = L2_kinematics.getPdCurrent()
    voltageread = L1_ina.readVolts()
    L1_log.uniqueFile(voltageread,"voltage")
    L1_log.uniqueFile(xdot[0],"ForwardVelocity")
    print("Forward Vel: ",xdot[0])
    L1_log.uniqueFile(xdot[1],"AngularVelocity")
    print("Angular Vel: ",xdot[1])
    L1_log.uniqueFile(phis[0],"Leftmotor")
    print("Phi of Left: ",phis[0])
    L1_log.uniqueFile(phis[1],"Rightmotor")
    print("Phi of right: ",phis[1])
    time.sleep(.2)