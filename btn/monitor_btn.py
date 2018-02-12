#!/usr/bin/python
from evdev import InputDevice
from select import select
import os
dev = InputDevice('/dev/input/event0')
 
while True:
    r,w,x = select([dev], [], [])    
    for event in dev.read():            
        print(event)
        os.system('cd /home/crazypi/apps/wifi; ./setwifi > log_setwifi 2>&1')

