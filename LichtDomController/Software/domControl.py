#!/bin/python
# -*- coding: utf-8 -*-

# Programm/Daemon zur IO-Kontrolle des CamDoms

import os
from time import sleep
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106

import spidev

import ctypes
c_uint8 = ctypes.c_uint8

import RPi.GPIO as gpio

import SocketServer


serial = i2c(port=1, address=0x3C)
oled = ssd1306(serial, rotate=2)
speicher = ""

# SPI
class ledstruktur(ctypes.LittleEndianStructure):
    _fields_ =[("O45R", c_uint8, 1),
               ("O45G", c_uint8, 1),
               ("S45R", c_uint8, 1),
               ("S45G", c_uint8, 1),
               ("S90R", c_uint8, 1),
               ("S90G", c_uint8, 1),
               ("W90R", c_uint8, 1),
               ("W90G", c_uint8, 1),
               ("W45R", c_uint8, 1),
               ("W45G", c_uint8, 1),
               ("N90R", c_uint8, 1),
               ("N90G", c_uint8, 1),
               ("O90R", c_uint8, 1),
               ("O90G", c_uint8, 1),
               ("N45R", c_uint8, 1),
               ("N45G", c_uint8, 1),
               ("O00R", c_uint8, 1),
               ("O00G", c_uint8, 1),
               ("W00R", c_uint8, 1),
               ("W00G", c_uint8, 1)]

class Flags(ctypes.Union):  _fields_ = [("struktur", ledstruktur), ("asUint8", c_uint8 * 3)]

leds = Flags()
#leds.struktur.O45G = 1




def readRam():
    with open('/proc/meminfo') as file:
        for line in file:
            if "MemFree" in line:
                freeRam = (int(line.split()[1])/1024)
                #print "Freier Arbeitsspeicher:",freeRam,"MB"
                return freeRam

def readSD():
    speicher = os.popen("df").readlines()
    for line in speicher:
        if "/dev/root" in line:
            freeSd = (int(line.split()[3])/1024)
            #print "Freier SD-Kartenspeicher:",freeSd,"MB"
            return freeSd

def readIP():
    adresse = os.popen("ip addr show eth0 | grep \"inet \" | awk '{print $2}' | cut -d/ -f1").read()
    return adresse

def readCoreTemp():
    temperatur = os.popen("vcgencmd measure_temp").read()
    temperatur = temperatur[5:]
    return temperatur

def printOled(ip, rom, ram, th, status=0):
    with canvas(oled) as draw:
        draw.rectangle(oled.bounding_box, outline="white", fill="black")
        if status == 0:
            draw.text((5, 5), "KameraDOM offline", fill="white")
        else:
            draw.text((5, 5), "KameraDOM online", fill="white")

        freeRam = "free RAM: " + str(ram) + " MB"
        freeRom = "free SD:  " + str(rom) + " MB"
        ip = "IP: " + ip
        th = "CPU-Temp: " + th

        draw.text((5, 15), freeRam, fill="white")
        draw.text((5, 25), freeRom, fill="white")
        draw.text((5, 35), ip, fill="white")
        draw.text((5, 45), th, fill="white")

def controlHW(controlword):
    # Unterscheidung des Testwortes nach Anfangsbuchstaben, dieser ist Eindeutig!
    laenge = len(controlword)
    ende = controlword[laenge-1:laenge]
    if ende != "\r\n":
        # Fehler! es ist ausgemacht, dass ein Controlstring mit \r\n endet.
        print("FEHLER! - falsches Zeilenende des Lampenkontrollworts")
        return

    if controlword[1] == 'R':
        print ("ROT")

    elif controlword[1] == 'G':
        print ("GRÜN")

    elif controlword[1] == 'W':
        print ("WS2812")

    elif controlword[1] == 'F':
        print ("Lüfter")



def disableAllLeds():
    # alles aus ist Stumpf 24 Bit Nullen
    spi.xfer([0,0,0])

def testLeds():
    for i in range(1, 10):
        spi.xfer([0x04, 0x00, 0x00])
        sleep(0.5)
        spi.xfer([0x04, 0x00, 0x01])
        sleep(0.5)

def controlFan(anaus = 0):
    if anaus == 0:
        gpio.output(17, gpio.LOW)
    elif anaus == 1:
        gpio.output(17, gpio.HIGH)


class MyTCPHandler(SocketServer.StreamRequestHandler):

    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)

        controlHW(self.data)
        # Likewise, self.wfile is a file-like object used to write back
        # to the client
#        self.wfile.write(self.data.upper())



if __name__ == '__main__':
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 10000000
    spi.mode = 0

    gpio.setmode(gpio.BCM)
    gpio.setup(17, gpio.OUT, initial=gpio.LOW)

    HOST, PORT = "0.0.0.0", 1337

    # Create the server, binding to localhost on port 1337
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

    print "mem:", readRam(), "MB"
    print "sd :", readSD(), "MB"
    print "IP: ", readIP()
    print "Core-Temperatur: ", readCoreTemp()
    printOled(readIP(), readSD(), readRam(), readCoreTemp(), 1)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()

    gpio.output(17, gpio.LOW)
    disableAllLeds()
