#! /usr/bin/env python
#coding=utf-8

from miio import fan
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
ip = "192.168.31.197";
token = "acc7224feccc637e135f4655ce0c305a"
fan = fan.FanP5(ip, token)

while 1:
    print("IN LOOP!")
    fan.on()
    fan.set_natural_speed(80)
    time.sleep(2)
    plug.off()
    time.sleep(2)
