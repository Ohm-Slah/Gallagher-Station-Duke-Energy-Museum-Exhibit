#
# File name:         "playvideo.py"
# Contributor(s):    Elliot Eickholtz, Andrew Boehm
# Last edit:         3/23/22
# Code usage:
# This code is intended to run on a Raspberry Pi with a USB serial connection to the Arduino Mega
# When a predefined serial command is recieved, the appropriate video and audio will play
# on the HDMI-connected screen from the Raspberry Pi using

# Import all libraries needed
# import sys                      # library for controlling System-specific parameters and functions
# import os                       # library of Miscellaneous operating system interfaces
# from subprocess import Popen    # library allows for spawning new processes
# import psutil                   # library for retrieving information on running processes and system utilization
import serial                   # library for basic serial communication
import time                     # library for a delay
import vlc


previousCommand = "NONE"
s = ""       #used fo serial
serialFlagEvent = 0

player = vlc.Instance("--verbose=0", "--vout=any")

media_player = player.media_player_new()

#deepsleep = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/DEEPSLEEP.mov")
phasezero = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase0.mp4")
intro = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/IntroVideo.mp4")

phaseoneintro = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase1intro.mp4")
phaseoneloop = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase1instruct.mp4")
phaseonefailhigh = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase1high.mp4")
phaseonefaillow = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase1low.mp4")
#phaseoneunbalanced = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/FAILPHASE1UNSTABLE.mov")

phasetwointro = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase2intro.mp4")
phasetwoloop = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase2intruct-1.mp4")
phasetwofailhigh = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase2high.mp4")
phasetwofaillow = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase2low.mp4")

phasethreeintro = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase3intro.mp4")
phasethreeloop = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase3instruct.mp4")
phasethreefailhigh = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase3high.mp4")
phasethreefaillow = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase3low.mp4")

phasefourintro = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase4intro.mp4")
phasefourloop = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phase4instruct.mp4")
phasefourfailhigh = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase4high.mp4")
phasefourfaillow = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Failphase4low.mp4")

ring = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Phongring.mp4")
complete = player.media_new("/home/pi/Desktop/Gallagher-Station-Duke-Energy-Museum-Exhibit-main/pi/videos/Success.mp4")

# link variable name to serial instance on raspberry pi
try:
    ser = serial.Serial('/dev/ttyACM1')
except:
    ser = serial.Serial('/dev/ttyACM0')

def loopVideoUntilEvent():
    global serialFlagEvent
    while(not ser.in_waiting):
        while (media_player.get_position()<0.95):
            if(ser.in_waiting): serialFilter()
            if(serialFlagEvent): break
            print("lVUE() WAIT")
            print(media_player.get_position())
            print(media_player.get_state())
            time.sleep(1)
        media_player.set_position(0)
    print("EVENT: END VIDEO")
            
def waitUntilVideoEnd():
    global serialFlagEvent
    media_player.set_position(0)
    while (media_player.get_position()<0.93):
        print("wUVE() WAIT")
        if(ser.in_waiting):
            print("SERIAL IN BUFFER")
            serialFilter()
        if(serialFlagEvent):
            print("FLAG BREAK")
            break
        print(media_player.get_position())
        time.sleep(1)
    print("END VIDEO")

def serialFilter():
    global previousCommand
    global s
    global serialFlagEvent
    
    if ser.in_waiting:
        s = ser.readline(100).decode()
        print("GOT:")
        print(s)
            
        if (s in previousCommand) or (s == previousCommand):
            return
        else:
            serialFlagEvent = 1
            previousCommand = s


# Run this forever
while True:
    # error handlin
    try:
        serialFilter()
        if (media_player.get_position() == -1) and previousCommand!="NONE" and media_player.get_state() == 'State.Playing':
            serialFlagEvent = 1
            s = previousCommand
        
        if(serialFlagEvent):
            serialFlagEvent = 0
            # check each predefined string and play its corresponding video
            # send confirmation number back through serial
            if "RESPOND" in s:
                print('response request')
                ser.write('1'.encode())
            
            elif "INTROS" in s:
                ser.write('2'.encode())
                print("main intro video")
                media_player.set_media(intro)
                media_player.play()
                #time.sleep(1)
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()

            
            elif "PHASE ZERO" in s:
                ser.write('3'.encode())
                print("phase zero video")
                media_player.set_media(phasezero)
                media_player.play()
                media_player.set_fullscreen(1)
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "PHASE ONE INTRO" in s:
                ser.write('5'.encode())
                print("phase one intro video")
                media_player.set_media(phaseoneintro)
                media_player.play()
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()
            
            elif "PHASE ONE LOOP" in s:
                ser.write('6'.encode())
                print("phase one loop")
                media_player.set_media(phaseoneloop)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()


            elif "PHASE ONE FAIL HIGH" in s:
                ser.write('7'.encode())
                print("phase one fail high")
                media_player.set_media(phasezero)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()

            elif "PHASE ONE FAIL LOW" in s:
                ser.write('8'.encode())
                print("phase one fail low")
                media_player.set_media(phaseonefaillow)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
# 
#             elif "PHASE ONE UNBALANCED" in s:
#                 ser.write('9'.encode())
#                 print("phase one unbalanced")
#                 media_player.set_media(phaseoneunbalanced)
#                 media_player.play()
#                 loopVideoUntilEvent()
#                 media_player.stop()
            
            elif "PHASE TWO INTRO" in s:
                print("phase two intro")
                ser.write('A'.encode())
                media_player.set_media(phasetwointro)
                media_player.play()
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()
            
            elif "PHASE TWO LOOP" in s:
                print("phase two loop")
                ser.write('B'.encode())
                media_player.set_media(phasetwoloop)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
               
                #startVideo(phasetwoloop, n)
                #phasetwoloop.set_fullscreen(True)
                #media_player.play_item_at_index(9)
            
            elif "PHASE TWO FAIL HIGH" in s:
                print("phase two fail high")
                ser.write('C'.encode())
                media_player.set_media(phasetwofailhigh)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "PHASE TWO FAIL LOW" in s:
                print("phase two fail low")
                ser.write('D'.encode())
                media_player.set_media(phasetwofaillow)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "PHASE THREE INTRO" in s:
                print("phase three intro")
                ser.write('E'.encode())
                media_player.set_media(phasethreeintro)
                media_player.play()
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()
                
            elif "PHASE THREE LOOP" in s:
                print("phase three loop")
                ser.write('F'.encode())
                media_player.set_media(phasethreeloop)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
               
                #startVideo(phasethreeloop, n)
                #phasethreeloop.set_fullscreen(True)
                #media_player.play_item_at_index(13)
            
            elif "PHASE THREE FAIL HIGH" in s:
                print("phase three fail high")
                ser.write('G'.encode())
                media_player.set_media(phasethreefailhigh)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
                
            elif "PHASE THREE FAIL LOW" in s:
                print("phase three fail low")
                ser.write('H'.encode())
                media_player.set_media(phasethreefaillow)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
                
            
            elif "PHASE FOUR INTRO" in s:
                print("phase four intro")
                ser.write('I'.encode())
                media_player.set_media(phasefourintro)
                media_player.play()
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()
            
            elif "PHASE FOUR LOOP" in s:
                print("phase four loop")
                ser.write('J'.encode())
                media_player.set_media(phasefourloop)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "PHASE FOUR FAIL HIGH" in s:
                print("phase four fail high")
                ser.write('K'.encode())
                media_player.set_media(phasefourfailhigh)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "PHASE FOUR FAIL LOW" in s:
                print("phase four fail low")
                ser.write('L'.encode())
                media_player.set_media(phasefourfaillow)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()

            elif "RING" in s:
                print("ring phone")
                ser.write('N'.encode())
                media_player.set_media(ring)
                media_player.play()
                loopVideoUntilEvent()
                media_player.stop()
            
            elif "CRITICAL ERROR" in s:
                print("Error, resetting...")
                ser.write('1'.encode())
            
            elif "COMPLETE" in s:
                print("completion")
                ser.write('M'.encode())
                media_player.set_media(complete)
                media_player.play()
                waitUntilVideoEnd()
                ser.write('0'.encode())
                media_player.stop()
                
            elif "WAIT" in s:
                print("wait")
                if media_player.is_playing():
                    ser.write('0'.encode())
                else:
                    ser.write('2'.encode())


    # catch all errors, print error and continue running
    except Exception as e:
        print("ERROR : "+str(e))
        exit()
