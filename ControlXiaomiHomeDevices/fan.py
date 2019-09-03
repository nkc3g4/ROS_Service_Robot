#! /usr/bin/env python
#coding=utf-8

from miio import fan
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
ip = "192.168.28.34"
token = "c4ab4e188fb428a557e02dca95c028e5"
ffan = fan.FanP5(ip, token)

print("IN LOOP!")
ffan.on()
time.sleep(2)
ffan.on()
#fan.set_natural_speed(80)
time.sleep(4)
ffan.off()
time.sleep(2)
ffan.off()
time.sleep(4)
