#
# File name:         "playvideo.py"
# Contributor(s):    Elliot Eickholtz
# Last edit:         3/16/22
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
deepsleep = "/home/pi/Desktop/main/data/DEEPSLEEP.mov"
phasezero = "/home/pi/Desktop/main/data/PHASE0.mov"

phaseoneintro = "/home/pi/Desktop/main/data/PHASE1NITRO.mov"
phaseoneloop = "/home/pi/Desktop/main/data/PHASE1LOOP.mov"
phaseonefailhigh = "/home/pi/Desktop/main/data/PHASE1FH.mov"
phaseonefaillow = "/home/pi/Desktop/main/data/PHASE1FL.mov"
phaseoneunbalanced = "/home/pi/Desktop/main/data/PHASE1UNB.mov"

phasetwointro = "/home/pi/Desktop/main/data/PHASE1NITRO.mov"
phasetwoloop = "/home/pi/Desktop/main/data/PHASE1LOOP.mov"
phasetwofailhigh = "/home/pi/Desktop/main/data/PHASE2FH.mov"
phasetwofaillow = "/home/pi/Desktop/main/data/PHASE2FL.mov"

phasethreeintro = "/home/pi/Desktop/main/data/PHASE1NITRO.mov"
phasethreeloop = "/home/pi/Desktop/main/data/PHASE1LOOP.mov"
phasethreefailhigh = "/home/pi/Desktop/main/data/PHASE3FH.mov"
phasethreefaillow = "/home/pi/Desktop/main/data/PHASE3FL.mov"

phasefourintro = "/home/pi/Desktop/main/data/PHASE1NITRO.mov"
phasefourloop = "/home/pi/Desktop/main/data/PHASE1LOOP.mov"
phasefourfailhigh = "/home/pi/Desktop/main/data/PHASE4FH.mov"
phasefourfaillow = "/home/pi/Desktop/main/data/PHASEFL.mov"

ring = "/home/pi/Desktop/main/data/RING.mov"

complete = "/home/pi/Desktop/main/data/COMPLETE.mov"

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

            elif "DEEP SLEEP" in s:
                print("deep sleep")
                ser.write('1')
                startVideo(deepsleep, n)
            
            elif "PHASE ZERO" in s:
                print("phase zero")
                ser.write('1')
                startVideo(phasezero, n)
            
            elif "PHASE ONE INTRO" in s:
                print("phase one intro")
                ser.write('1')
                startVideo(phaseoneintro, n)
            
            elif "PHASE ONE LOOP" in s:
                print("phase one loop")
                ser.write('1')
                startVideo(phaseoneloop, n)

            elif "PHASE ONE FAIL HIGH" in s:
                print("phase one fail high")
                ser.write('1')
                startVideo(phaseonefailhigh, n)

            elif "PHASE ONE FAIL LOW" in s:
                print("phase one fail low")
                ser.write('1')
                startVideo(phaseonefaillow, n)

            elif "PHASE ONE UNBALANCED" in s:
                print("phase one unbalanced")
                ser.write('1')
                startVideo(phaseoneunbalanced, n)
            
            elif "PHASE TWO INTRO" in s:
                print("phase two intro")
                ser.write('1')
                startVideo(phasetwointro, n)
            
            elif "PHASE TWO LOOP" in s:
                print("phase two loop")
                ser.write('1')
                startVideo(phasetwoloop, n)
            
            elif "PHASE TWO FAIL HIGH" in s:
                print("phase two fail high")
                ser.write('1')
                startVideo(phasetwofailhigh, n)
            
            elif "PHASE TWO FAIL LOW" in s:
                print("phase two fail low")
                ser.write('1')
                startVideo(phasetwofaillow, n)
            
            elif "PHASE THREE INTRO" in s:
                print("phase three intro")
                ser.write('1')
                startVideo(phasethreeintro, n)
            
            elif "PHASE THREE LOOP" in s:
                print("phase three loop")
                ser.write('1')
                startVideo(phasethreeloop, n)
            
            elif "PHASE THREE FAIL HIGH" in s:
                print("phase three fail high")
                ser.write('1')
                startVideo(phasethreefailhigh, n)
            
            elif "PHASE THREE FAIL LOW" in s:
                print("phase three fail low")
                ser.write('1')
                startVideo(phasethreefaillow, n)
            
            elif "PHASE FOUR INTRO" in s:
                print("phase four intro")
                ser.write('1')
                startVideo(phasefourintro, n)
            
            elif "PHASE FOUR LOOOP" in s:
                print("phase four loop")
                ser.write('1')
                startVideo(phasefourloop, n)
            
            elif "PHASE FOUR FAIL HIGH" in s:
                print("phase four fail high")
                ser.write('1')
                startVideo(phasefourfailhigh, n)
            
            elif "PHASE FOUR FAIL LOW" in s:
                print("phase four fail low")
                ser.write('1')
                startVideo(phasefourfaillow, n)

            elif "RING" in s:
                print("ring phone")
                ser.write('1')
                startVideo(ring, n)
            
            elif "CRITICAL ERROR" in s:
                print("Error, resetting...")
                ser.write('1')
            
            elif "COMPLETE" in s:
                print("completion")
                ser.write('1')
                startVideo(complete, n)

    # catch all errors, print error and continue running
    except Exception as e:
        print("ERROR : "+str(e))
