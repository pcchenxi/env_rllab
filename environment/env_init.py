#!/usr/bin/env python
from environment.vrep_plugin import vrep
import time
def connect_vrep(port_num):

    clientID = vrep.simxStart('127.0.0.1', port_num, True, True, 5000, 5)
    if clientID != -1:
        print ('Connected to remote API server with port: ', port_num)
    else:
        print ('Failed connecting to remote API server with port: ', port_num)

    print (clientID)
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    time.sleep(1)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    time.sleep(1)

def init_environment(num):
    vrep.simxFinish(-1)
    for i in range(num):
        connect_vrep(20000 + i)