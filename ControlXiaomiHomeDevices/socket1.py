#! /usr/bin/env python
#coding=utf-8

from miio import chuangmi_plug
#from yeelight import discover_bulbs
import time

#print discover_bulbs()
ip = '192.168.28.114'
token = '7bebc798739c7e2640ffe4cc6568f8d3'

plug = chuangmi_plug.ChuangmiPlug(ip, token)

for i in range(5):
    print("IN LOOP!")
    plug.on()
    time.sleep(2)
    plug.off()
    time.sleep(2)

