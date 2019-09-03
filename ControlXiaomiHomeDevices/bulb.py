#! /usr/bin/env python
#coding=utf-8

from miio import yeelight
#from yeelight import Bulb
#from yeelight import discover_bulbs
import time

#print discover_bulbs()#,token = "4f49086a9f74e09a5d8698e3d5426d6e"
bulb = yeelight.Yeelight("192.168.28.227","725b43750ba04c97a130ac19cb949ff2")

for i in range(2):
        print("IN LOOP!")
        bulb.on()
        bulb.set_brightness(99)
        bulb.set_color_temp(6000)
        time.sleep(1)
        bulb.set_color_temp(4000)
        time.sleep(1)	
        bulb.off()
        time.sleep(2)
