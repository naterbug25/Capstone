# Nathan Huber
# 20181213
# Senior Design Project
# Rev: 1.0
# pycomm_Huber_20181213 

import sys
sys.path.append('/home/pi/pycomm/') # Python
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/pycomm2') # Windows
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/pycomm3') # Windows

from pycomm.ab_comm.clx import Driver as ClxDriver
import time
import logging
import random

def PI_PLC (Head_Angle_Current):
    PLC = ClxDriver()
    
    print("Opening Connection To PLC...")
    if PLC.open('192.168.1.150'):
        print("*********************************")
        PLC_Known_Head_Angle,_=PLC.read_tag("R_PI_User_Head_Angle")           # Read the current head angle the PLC knows of
        print("PLC_Known_Head_Angle: " + str(PLC_Known_Head_Angle))           # Print the current head angle the PLC knows of
        print("Head_Angle_Current: " + str(Head_Angle_Current))               # Print the head angle passed into this argument
        PLC.write_tag('R_PI_User_Head_Angle', Head_Angle_Current, 'INT')      # Tell the PLC the new angle
        PLC.write_tag('R_PI_Comm_Check', random.randint(-25,25), 'INT')       # Send a random value to the PLC to make sure data is received
        PLC_Known_Head_Angle,_=PLC.read_tag("R_PI_User_Head_Angle")           # Read the new angle
        print("NEW PLC_Known_Head_Angle: " + str(PLC_Known_Head_Angle))       # Print the current head angle the PLC knows of
        # Make sure the PLC Read the new angle
        if Head_Angle_Current != PLC_Known_Head_Angle:
            print("FAULT: PI -> PLC TRANSMISSION FAILED!!!")
            
        print("Closing Connection...")
        print("********************************* \n")
        PLC.close()
        
    PLC.close()
        
        
        
