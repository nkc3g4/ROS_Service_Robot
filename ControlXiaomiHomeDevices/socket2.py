#! /usr/bin/env python
#coding=utf-8

from miio import chuangmi_plug
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
plug = chuangmi_plug.ChuangmiPlug("192.168.28.229",'ede445d92432153996ae77bd7c3a1d5c')

while 1:
    print("IN LOOP!")
    #plug.on()
    #time.sleep(2)
    plug.off()
    time.sleep(1)
    plug.off()
