#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Programm/Daemon zur IO-Kontrolle des CamDoms

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
import luma.core.error
import spidev
import ctypes
import RPi.GPIO as gpio
import os
import socketserver
import threading
import queue
import logging
import logging.handlers
import argparse
c_uint8 = ctypes.c_uint8
# Definition of local classes
class Ledstruct(ctypes.LittleEndianStructure):
    _fields_ = [("O45R", c_uint8, 1),
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
class Ledunion(ctypes.Union):  _fields_ = [("struktur", Ledstruct), ("asUint8", c_uint8 * 3),
                                        ("asUint32", ctypes.c_uint32)]
led_lut = dict({ "O45R": 0,
            "O45G":     1,
            "S45R":     2,
            "S45G":     3,
            "S90R":     4,
            "S90G":     5,
            "W90R":     6,
            "W90G":     7,
            "W45R":     8,
            "W45G":     9,
            "N90R":     10,
            "N90G":     11,
            "O90R":     12,
            "O90G":     13,
            "N45R":     14,
            "N45G":     15,
            "O00R":     16,
            "O00G":     17,
            "W00R":     18,
            "W00G":     19})

# Init global logging handler to syslog

logger = logging.getLogger('sysLogLogger')
address = '/dev/log'
if not os.path.exists(address):
    address = 'localhost', 514
log_handler = logging.handlers.SysLogHandler(address=address)
logger.addHandler(log_handler)
log_handler.setLevel(logging.WARNING)

argparser = argparse.ArgumentParser(
    description=
    'TCP Handler for lightdom')
argparser.add_argument(
    '-v', dest='verbose', action='store_true', help="Verbose output")
argparser.add_argument(
    '-p',
    dest='port',
    type=int,
    default=1337,
    help='Port of tcp handler')
argparser.add_argument(
    '-i',
    dest='interface',
    default="0.0.0.0",
    help='Interface of tcp handler')
args = argparser.parse_args()

if args.verbose:
    log_handler.setLevel(logging.DEBUG)

# Create msq_queue from TCP server threads to main thread
msg_queue = queue.Queue()

leds = Ledunion()

def readRam():
    with open('/proc/meminfo') as file:
        for line in file:
            if "MemFree" in line:
                freeRam = (int(line.split()[1]) / 1024)
                # print "Freier Arbeitsspeicher:",freeRam,"MB"
                return freeRam


def readSD():
    speicher = os.popen("df").readlines()
    for line in speicher:
        if "/dev/root" in line:
            freeSd = (int(line.split()[3]) / 1024)
            # print "Freier SD-Kartenspeicher:",freeSd,"MB"
            return freeSd


def readIP():
    adresse = os.popen("ip addr show eth0 | grep \"inet \" | awk '{print $2}' | cut -d/ -f1").read()
    return adresse


def readCoreTemp():
    temperatur = os.popen("vcgencmd measure_temp").read()
    temperatur = temperatur[5:]
    return temperatur


def printOled(oled, ip, rom, ram, th, status=0):
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


def controlHW(spi, controlword):
    # Unterscheidung des Testwortes nach Anfangsbuchstaben, dieser ist Eindeutig!
    laenge = len(controlword)
    if laenge == 6:
        led_name = controlword[:4]
        if led_name in led_lut:
            if controlword[5] == '1':
                leds.asUint32 |= 1 << led_lut[led_name]
            else:
                leds.asUint32 &= ~(1 << led_lut[led_name])
            controlLeds(spi, leds)


def disableAllLeds(spi):
    # alles aus ist Stumpf 24 Bit Nullen
    leds.asUint32 = 0
    controlLeds(spi, leds)


def controlLeds(spi, led_struk):
    spi.xfer([led_struk.asUint8[2], led_struk.asUint8[1], led_struk.asUint8[0]])


def controlFan(enable):
    if enable is True:
        gpio.output(17, gpio.LOW)
    else:
        gpio.output(17, gpio.HIGH)


class NetworkHandler(socketserver.StreamRequestHandler):

    def handle(self):
        # MOTD
        self.wfile.write(bytearray("LightDOM V1\r\n", encoding='ascii'))
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        while True:
            data = self.rfile.readline().strip()
            logger.debug("Got msg with length".format(len(data)))
            if len(data) == 0:
                break
            logger.debug("{} wrote:".format(self.client_address[0]))
            hex_string = data.hex()
            logger.debug(hex_string)
            if len(data) < 3:
                logger.warning("Msg is too short")
                break
            # Prüfen des Längenbytes auf ASCII Zahlenbereich
            if not (0 < int(data[0]) < 10):
                logger.warning("Keine ASCII Zahl {} als Längenangabe Type: {}".format(data[0], type(data[0])))
                break
            if len(data) - 1 != (int(data[0])):
                logger.warning("Ungültige Längenangabe")
                break
            if data[1] != ',':
                logger.warning("Komma als Trennzeichen zwischen Länge und Nachricht nicht gefunden")
                break
            data = data[2:]
            msg_queue.put(data)


def shutdown(spi):
    disableAllLeds(spi)
    gpio.output(17, gpio.LOW)
    gpio.cleanup()
    server.shutdown()
    server.server_close()

def getSPIInterface():
    spi = None
    try:
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 10000000
        spi.mode = 0
    except Exception as e:
        logger.warning("Could not open SPI Interface")
        exit(-1)
    return spi

def getOledInterface():
    oled = None
    try:
        serial = i2c(port=1, address=0x3C)
        oled = ssd1306(serial, rotate=2)
    except luma.core.error:
        logger.warning("Could not connect to i2c oled display")
        exit(-1)
    return oled


if __name__ == '__main__':

    spi_interface = getSPIInterface()
    oled_interface = getOledInterface()

    gpio.setmode(gpio.BCM)
    gpio.setup(17, gpio.OUT, initial=gpio.LOW)

    # Create the server, binding to localhost on port 1337
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    server = socketserver.ThreadingTCPServer((args.interface, args.port), NetworkHandler)
    printOled(oled_interface, readIP(), readSD(), readRam(), readCoreTemp(), 1)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = False
    server_thread.start()
    while True:
        try:
            item = msg_queue.get(block=True, timeout=500)
            controlHW(spi_interface, item)
        except queue.Empty:
            printOled(oled_interface, readIP(), readSD(), readRam(), readCoreTemp(), 1)
        except KeyboardInterrupt:
            break
    shutdown(spi_interface)
