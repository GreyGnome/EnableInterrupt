## Arduino Makefile, see https://github.com/sudar/Arduino-Makefile (it's awesome!)
ARDUINO_DIR = /home/schwager/bin/arduino-1.8.5

# ISP_PORT          = /dev/ttyACM0
#BOARD_TAG         = attiny85at8
BOARD_TAG         = attiny44at8
ALTERNATE_CORE    = tiny
BOARDS_TXT = ~/sketchbook/hardware/arduino-tiny/boards.txt
ARDUINO_CORE_PATH = /home/schwager/sketchbook/hardware/arduino-tiny/cores/tiny
# ARDUINO_VAR_PATH  = ~/sketchbook/hardware/arduino-tiny/cores/tiny
#AVR_TOOLS_DIR = /home/schwager/bin/arduino-1.8.4/hardware/tools/avr

CFLAGS_STD = -g
CXXFLAGS_STD = -g

ISP_PROG = usbtiny
F_CPU = 1000000L

include /usr/share/arduino/Arduino.mk
