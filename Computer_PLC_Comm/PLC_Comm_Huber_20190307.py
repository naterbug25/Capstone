# Nathan Huber
# Started: 20181213
# Research
# Rev: 1.0
# PLC_Comm_Huber_20190307.py

# *********************************************************************
# Step 0: Append File Paths & Imports
# *********************************************************************

import sys
sys.path.append('/home/pi/pycomm/')
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/') # Windows
sys.path.append('/Users/hubern/Desktop/Nathan Huber Senior/Senior_Design/pycomm3') # Windows

# *********************************************************************
# Step 1: Initialize Global Variables
# *********************************************************************

from pycomm.ab_comm.clx import Driver as ClxDriver
import time
import logging
import random

# *********************************************************************
# Step 2: Main Routine
# *********************************************************************

def PI_PLC (X_CAM, Y_CAM, Z_CAM):
            
    # *********************************************************************
    # Step 3: Print X, Y, Z values passed into routine 
    # *********************************************************************     

    print("X_CAM: %s" % (X_CAM))                                           # Print the value passed
    print("Y_CAM: %s"% (Y_CAM))                                             # Print the value passed
    print("Z_CAM: %s" % (Z_CAM))                                           # Print the value passed
        
    PLC = ClxDriver()

    # *********************************************************************
    # Step 4: Connect to PLC
    # *********************************************************************     

    print("Opening Connection To PLC...")
    if PLC.open('192.168.1.150'):
        print("*********************************")
         
        # *********************************************************************
        # Step 5: Write Data to PLC
        # *********************************************************************  

		# Write Data
        PLC.write_tag('X_CAM', int(X_CAM), 'INT')                              # Tell the PLC the new value
        PLC.write_tag('Y_CAM', int(Y_CAM), 'INT')                              # Tell the PLC the new value
        PLC.write_tag('Z_CAM', int(Z_CAM), 'INT')                              # Tell the PLC the new value       

        # *********************************************************************
        # Step 6: Read Data PLC Received
        # *********************************************************************  

		# Read tags
        X_CAM_PLC,_=PLC.read_tag("X_CAM")                                     # Read the value the PLC knows of
        Y_CAM_PLC,_=PLC.read_tag("Y_CAM")                                     # Read the value the PLC knows of
        Z_CAM_PLC,_=PLC.read_tag("Z_CAM")                                     # Read the value the PLC knows of
        # Print tags
        print("X_CAM_PLC: %s" % (X_CAM_PLC))                              # Print the current value the PLC knows of
        print("Y_CAM_PLC: %s" % (Y_CAM_PLC))                              # Print the current value the PLC knows of
        print("Z_CAM_PLC: %s" % (Z_CAM_PLC))                               # Print the current value the PLC knows of
		
        # *********************************************************************
        # Step 7: Close connection to PLC
        # *********************************************************************  

        print("Closing Connection...")
        print("********************************* \n")
        PLC.close()
        
    PLC.close()
        
        
        
        
