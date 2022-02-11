# Gallagher-Station-Museum-Exhibit
![GitHub repo size](https://img.shields.io/github/repo-size/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit) ![Lines of code](https://img.shields.io/tokei/lines/github/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit)

A code repository for the Duke Energy funded exhibit at the Padgett Museum of the Gallagher Station, created by Purdue Polytechnic SoET students. This repository is only for the use of and contributed by these students. However, this information is for public viewing and usage. Any unwarranted pull-requests from unathorized individuals will be rejected, as this is to document the final product as-is. All code is under no licensure and is open for usage in the public domain. 

![0607_ky_gallagher_station_plant](https://user-images.githubusercontent.com/72700028/139541656-05c52c4c-2b8d-4746-b4ec-811eb962cbc5.jpg)

# What is this?
The Duke Energy Gallagher Station is located in New Albany, IN, and was a power generating facility that was of tremendous aid in powering Southern Indiana from 1958 until decommissioning in June of 2021. Duke Energy proposed the creation of an interactive museum exhibit representing the operation of a coal-fired power plant, with focus on the control system within the display. 

This project will be located at the Padgett Museum in New Albany, IN. The showcase will educate users on energy production control using older and current technologies and incorporating donated equipment from the Gallagher Station. The interactive exhibit will primarily be used by senior museum guests during normal operating hours of the Padgett Museum for an average of XX amount of time, and will not require training or previous knowledge of power systems to interact with it. This project must safely fit into the designated area of the museum without obstructing walking paths or needing to modify the surrounding environment. This project must be capable of being moved to a school and interacted with by a younger audience.

The code running on the Arduino Mega was written on a VSCode environment compiled and linked with the extension PlatformIO.

The code running on the ATTiny4313 controlling the 7-segment diplay is writen, compiled, and linked in the Arduino IDE.

# Folder Structure
The [src folder](https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/tree/main/src) contains all cpp files to run on the Arduino Mega.

The [include folder](https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/tree/main/include) contains all c header files used on Arduino Mega.

The [pi folder](https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/tree/main/pi) contains the python code running on the raspberry pi.

The [pictures folder](https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/tree/main/Pictures) contains pictures used in this repository.

The proc folder is no longer used, but is there for reference from another version of code on the Raspberry Pi.

# Proof of Concept Prototype
The proof of [concept prototype code](https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/tree/Proof-of-Concept-Prototype) has been left as a seperate branch in Github, and you can view the project as how it was before any polish whatsoever. In the proof of concept for the method of playing videos at the request of a microcontroller, the program chosen initially was processing3 on Raspberry Pi. This was written in Java and worked well on a PC, however when running on a Raspberry Pi, the framerate dipped drastically. After a great deal of debugging and research, the final decision was to use OMXPlayer on Raspberry Pi programmed in Python. This switch to OMXPlayer was not realized until after the proof of concept prototype.
The proof of concept was also only programmed initially to showcase a rudimentary version of phases 0, 1, & 2, as well as the failure and completion routines. This code also showed the functionality of a stepper motor designed into a donated Gallagher Station gauge in place of a phase 3. There was no phase 4 written.
