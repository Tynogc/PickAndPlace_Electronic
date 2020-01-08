#!/bin/python
# -*- coding: utf-8 -*-

# Programm/Daemon zur IO-Kontrolle des CamDoms

import os
from time import sleep
from luma.core.interface.serial import i2c, spi
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106

serial = i2c(port=1, address=0x3C)
oled = ssd1306(serial, rotate=2)
speicher = ""

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


if __name__ == '__main__':

    print "mem:", readRam(), "MB"
    print "sd :", readSD(), "MB"
    print "IP: ", readIP()
    print "Core-Temperatur: ", readCoreTemp()
    printOled(readIP(), readSD(), readRam(), readCoreTemp(), 1)

    sleep(10)