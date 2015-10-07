## Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile
#
#AVR_TOOLS_DIR = /usr
#CFLAGS_STD = -g -I /home/schwager/.arduino15/packages/attiny/hardware/avr/1.0.1/variants/tiny8
#CXXFLAGS_STD = -g -I /home/schwager/.arduino15/packages/attiny/hardware/avr/1.0.1/variants/tiny8
#
## if you have placed the alternate core in your sketchbook directory, then you can just mention the core name alone.
#ALTERNATE_CORE = attiny
## If not, you might have to include the full path.
##ALTERNATE_CORE_PATH = /home/schwager/bin/arduino-1.6.4/hardware/arduino/avr/cores/
#ALTERNATE_CORE_PATH = /home/schwager/.arduino15/packages/attiny/hardware/avr/1.0.1
#ARDUINO_VAR_PATH = /home/schwager/Projects/Arduino/attiny-ide-1.6.x/attiny/avr/variants
#ARDUINO_HEADER = /home/schwager/bin/arduino-1.6.4/hardware/arduino/avr/cores/arduino/Arduino.h
#
#BOARD_TAG    = attiny85
#BOARD_SUB = attiny85
#ALTERNATE_CORE = attiny
#
#ISP_PROG = usbasp
#ISP_PORT = /dev/ttyACM*
#
#F_CPU = 1000000L
#
##include $(ARDMK_DIR)/Arduino.mk  # the variable is unrecognized.
#include /usr/share/arduino/Arduino.mk
#
## !!! Important. You have to use make ispload to upload when using ISP programmer
#ARDUINO_DIR = /home/schwager/bin/arduino-1.6.4/
#F_CPU = 16000000L
#
#include /usr/share/arduino/Arduino.mk
#

# NOTE: DAVID MELLIS' CODE INCLUDES ATTINY44/45/84/85 ONLY- NOT ATTINY24
# (that said, you *could* include the appropriate lines in the boards.txt file to
# support the smaller 'tinies).
ARDUINO_DIR = /home/schwager/bin/arduino-1.6.4
# 14-pin, ATtiny44/84
#BOARD_SUB = attiny44
#CFLAGS_STD = -g -I /home/schwager/sketchbook/hardware/attiny/variants/tiny14
#CXXFLAGS_STD = -g -I /home/schwager/sketchbook/hardware/attiny/variants/tiny14
#  

# 8-pin, ATtiny45/85
BOARD_SUB = attiny85
CFLAGS_STD = -g -I /home/schwager/sketchbook/hardware/attiny/variants/tiny8
CXXFLAGS_STD = -g -I /home/schwager/sketchbook/hardware/attiny/variants/tiny8

ISP_PROG = usbasp
BOARD_TAG = attiny
ALTERNATE_CORE = attiny
# Set this as appropriate. See the data sheet.
F_CPU = 1000000L

include /usr/share/arduino/Arduino.mk
