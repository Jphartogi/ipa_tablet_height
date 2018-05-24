#!/usr/bin/env python
import time
from roboclaw_driver.roboclaw import Roboclaw
#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

rc.ForwardMixed(address, 0)
rc.TurnRightMixed(address, 0)

while(1):
	rc.ForwardMixed(address, 16)
	time.sleep(5)
	rc.BackwardMixed(address, 0)
	time.sleep(5)
