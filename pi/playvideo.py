#
# File name:         "playvideo.py"
# Contributor(s):    Elliot Eickholtz
# Last edit:         2/9/22
# Code usage:
# This code is intended to run on a Raspberry Pi with a USB serial connection to the Arduino Mega
# When a predefined serial command is recieved, the appropriate video and audio will play
# on the HDMI-connected screen from the Raspberry Pi using OMXPlayer.

# Import all libraries needed
import sys                      # library for controlling System-specific parameters and functions
import os                       # library of Miscellaneous operating system interfaces
from subprocess import Popen    # library allows for spawning new processes
import psutil                   # library for retrieving information on running processes and system utilization
import serial                   # library for basic serial communication
import time                     # library for a delay

# link phase videos file location to variables
phasezero = "/home/pi/Desktop/main/data/PHASE0.mov"
phaseone = "/home/pi/Desktop/main/data/PHASE1.mov"
phasetwo = "/home/pi/Desktop/main/data/PHASE2.mov"
phasethree = "/home/pi/Desktop/main/data/PHASE3.mov"
phasefour = "/home/pi/Desktop/main/data/PHASE4.mov"
complete = "/home/pi/Desktop/main/data/COMPLETE.mov"
fail = "/home/pi/Desktop/main/data/FAIL.mov"

# link variable name to serial instance on raspberry pi
# this is actually changing every now and then when power cycled between 2 locations
# a solution must be found
ser = serial.Serial('/dev/ttyACM1')

# variable to hold active video player instances open
players = []

# variable to hold current layer number
n = 0

#print(ser.name)    # uncomment for debug

FNULL = open(os.devnull,'w')    # used by next line
#sys.stdout = FNULL             # uncomment this line to disable serial

# This function retrieves the number of currently active units of omxplayer,
# and stores them into an array called 'procs' and returns this variable
def getplayers():
    procs = []
    for p in psutil.process_iter():
        if p.name() == 'omxplayer.bin':
            procs.append(p)
    return procs

# This function grabs all of the processes found in getPlayers() except for the most
# recently started process and kills them.
def killoldplayers(procs):

    # array to collect processes' start times in milliseconds since epoch
    times = []

    # only run if more than one process open of omxplayer
    if len(procs) > 1:
        time.sleep(0.5)

        # collect times of processes
        for p in procs:
            times.append(p.create_time())
    
        # initialize variables to find the largest time (youngest task)
        largest_time = 0
        i = 0

        #print("TIMES : "+str(times))                       # uncomment for debug
        #print("len of procs before : "+str(len(procs)))    # uncomment for debug

        # find largest time
        for t in times:
            if t > largest_time:
                largest_time = i
            i+=1

        #print("largest time location : "+str(largest_time))   # uncomment for debug

        # remove newest time from kill list
        procs.pop(largest_time)
        #print(procs)                                          # uncomment for debug

        # kill all tasks in procs
        for p in procs:
            p.kill()
                
        #print("len of procs after : "+str(len(getplayers())))# uncomment for debug

# This function begins the video name sent to it at the layer number sent as well
def startVideo(vidName, layer):
    Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(layer), "%s"%(vidName)])


# Run this forever
while True:
    # error handling
    try:

        # restrict the existence of only one instance of omxplayer
        players = getplayers()
        killoldplayers(players)

        if ser.in_waiting:
            s = ser.readline(100)
            print(s)

            # iterate omxplayer layer count
            n += 1

            # check each predefined string and playits corresponding video
            # send confirmation '1' back through serial
            if "RESPOND" in s:
                print('response request')
                ser.write('1')
            
            elif "INTRO" in s:
                print("intro video")
                ser.write('1')
            
            elif "PHASE ZERO" in s:
                print("phase zero")
                ser.write('1')
                startVideo(phasezero, n)
            
            elif "PHASE ONE" in s:
                print("phase one")
                ser.write('1')
                startVideo(phaseone, n)
            
            elif "PHASE TWO" in s:
                print("phase two")
                ser.write('1')
                startVideo(phasetwo, n)
            
            elif "PHASE THREE" in s:
                print("phase three")
                ser.write('1')
                startVideo(phasethree, n)
            
            elif "PHASE FOUR" in s:
                print("phase four")
                ser.write('1')
                startVideo(phasefour, n)
            
            elif "CRITICAL ERROR" in s:
                print("Error, resetting...")
                ser.write('1')
            
            elif "COMPLETE" in s:
                print("completion")
                ser.write('1')
                startVideo(complete, n)
            
            elif "FAILURE" in s:
                print("fail")
                ser.write('1')
            
            elif "RING" in s:
                print("ring phone")
                ser.write('1')
                startVideo(fail, n)

    # catch all errors, print error and continue running
    except Exception as e:
        print("ERROR : "+str(e))
