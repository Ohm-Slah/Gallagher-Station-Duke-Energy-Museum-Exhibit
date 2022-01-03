import sys
import os
from subprocess import Popen
import psutil
import serial
#
movie1 = "/home/pi/Desktop/TEST.mp4"
phasezero = "/home/pi/Desktop/main/data/PHASE0.mov"
phaseone = "/home/pi/Desktop/main/data/PHASE1.mov"
phasetwo = "/home/pi/Desktop/main/data/PHASE2.mov"
complete = "/home/pi/Desktop/main/data/COMPLETE.mov"
fail = "/home/pi/Desktop/main/data/FAIL.mov"

ser = serial.Serial('/dev/ttyACM0')

print(ser.name)

n = 0
FNULL = open(os.devnull,'w')

def getplayers():
    procs = []
    for p in psutil.process_iter():
        if p.name() == 'omxplayer.bin':
            procs.append(p)
    return procs

def killoldplayers(procs):
    for p in procs:
        p.kill()

while True:
    players = getplayers()
    
    if ser.in_waiting:
        s = ser.readline(100)
        print(s)
        n += 1
        if "RESPOND" in s:
            print('response request')
            ser.write('1')
        elif "INTRO" in s:
            print("intro video")
            ser.write('1')
        elif "PHASE ZERO" in s:
            print("phase zero")
            ser.write('1')
            #cmd = "omxplayer -o local --layer %d %s "%(n,phasezero)
            Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(n), "%s"%(phasezero)])
            #os.system(cmd)
            killoldplayers(players)
        elif "PHASE ONE" in s:
            print("phase one")
            ser.write('1')
            Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(n), "%s"%(phaseone)])
            #killoldplayers(players)
        elif "PHASE TWO" in s:
            print("phase two")
            ser.write('1')
            Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(n), "%s"%(phasetwo)])
            #killoldplayers(players)
        elif "PHASE THREE" in s:
            print("phase three")
            ser.write('1')
        elif "PHASE FOUR" in s:
            print("phase four")
            ser.write('1')
        elif "CRITICAL ERROR" in s:
            print("Error, resetting...")
            ser.write('1')
        elif "COMPLETE" in s:
            print("completion")
            ser.write('1')
            Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(n), "%s"%(complete)])
        elif "FAILURE" in s:
            print("fail")
            ser.write('1')
        elif "RING" in s:
            print("ring phone")
            ser.write('1')
            Popen(['omxplayer', "--adev", "local", "--loop", "--layer","%d"%(n), "%s"%(fail)])
        killoldplayers(players)
    

