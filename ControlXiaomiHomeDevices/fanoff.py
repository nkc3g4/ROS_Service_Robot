#! /usr/bin/env python
#coding=utf-8

from miio import fan
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
ip = "192.168.31.197";
token = "fa1230c35f139885be7da59b4db2f174"
fan = fan.FanP5(ip, token)

print("IN LOOP!")
fan.off()

