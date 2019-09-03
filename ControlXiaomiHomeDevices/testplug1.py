#! /usr/bin/env python
#coding=utf-8

from miio import chuangmi_plug
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
ip = ''
token = ''

plug = chuangmi_plug.ChuangmiPlug("192.168.43.30",'5339aae5464ac87cdf5686ab0d8f75ce')

while 1:
    print("IN LOOP!")
    plug.on()
    time.sleep(2)
    plug.off()
    time.sleep(2)
