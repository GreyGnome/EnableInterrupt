#!/bin/bash

examples="
AllPins2560 
AllPins328 
ATtinyBlink 
HiSpeed 
HiSpeedAllPins2560 
HiSpeedAllPins328 
InterruptedPin2560 
InterruptedPin328
Mighty1284p
OOSimple
Simple
SimpleWithLibrary
"

for example in $examples; do
	echo $example
	sleep 2
	( cd $example; rm -rf build-*; make)
	echo $example done
	sleep 2
done
