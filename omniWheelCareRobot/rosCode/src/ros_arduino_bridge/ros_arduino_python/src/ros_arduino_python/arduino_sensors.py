#!/usr/bin/env python2

"""
    Sensor class for the arudino_python package

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
from decimal import Decimal
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1

class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5

class Sensor(object):
    def __init__(self, controller, name, pin, rate, frame_id, direction="input", **kwargs):
        self.controller = controller
        self.name = name
        self.pin = pin
        self.rate = rate
        self.direction = direction

        self.frame_id = frame_id
        self.value = None

        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":
                try:
                    self.value = self.read_value()
                except:
                    return
            else:
                try:
                    self.ack = self.write_value()
                except:
                    return

            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)

            self.t_next = now + self.t_delta

class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.ANALOG

        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher("~sensor/" + self.name, Analog, queue_size=5)

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        return self.controller.analog_read(self.pin)

    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)

class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.ANALOG

        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher("~sensor/" + self.name, AnalogFloat, queue_size=5)

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        return self.controller.analog_read(self.pin)

    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)


class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.BOOL

        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher("~sensor/" + self.name, Digital, queue_size=5)

        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW

    def read_value(self):
        return self.controller.digital_read(self.pin)

    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        self.value = not self.value
        return self.controller.digital_write(self.pin, self.value)

class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)

        self.message_type = MessageType.RANGE

        self.msg = Range()
        self.msg.header.frame_id = self.frame_id

        self.pub = rospy.Publisher("~sensor/" + self.name, Range, queue_size=5)

    def read_value(self):
        self.msg.header.stamp = rospy.Time.now()

class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)

        self.msg.radiation_type = Range.ULTRASOUND

class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)

        self.msg.radiation_type = Range.INFRARED

class Ping(SonarSensor):
    def __init__(self,*args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)

        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0

    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        cm = self.controller.ping(self.pin)

        # Convert it to meters for ROS
        distance = cm / 100.0

        return distance

class IR2Y0A02(IRSensor):
    def __init__(self, *args, **kwargs):
        super(IR2Y0A02, self).__init__(*args, **kwargs)

        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.200
        self.msg.max_range = 1.500

    def read_value(self):
        value = self.controller.analog_read(self.pin)

        try:
            volts = value*0.0048828125;
            distance = 65*pow(volts, -1.10)
        except:
            return self.msg.min_range

        # Convert to meters
        distance /= 100.000
        dist = round(float(distance), 3)

        # If we get a spurious reading, set it to the max_range
        if dist > self.msg.max_range: dist = self.msg.max_range
        if dist < self.msg.min_range: dist = self.msg.min_range

        return dist

class GP2Y0A41(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2Y0A41, self).__init__(*args, **kwargs)

        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.04
        self.msg.max_range = 0.30

    def read_value(self):
        value = self.controller.analog_read(self.pin)

        if value < 16:
            value = 16

        dist = 20.76/(value - 11.0)

        # If we get a spurious reading, set it to the max_range
        if dist > self.msg.max_range: dist = self.msg.max_range
        if dist < self.msg.min_range: dist = self.msg.min_range

        return Decimal(dist).quantize(Decimal('0.000'))

class GP2D12(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)

        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80

    def read_value(self):
        value = self.controller.analog_read(self.pin)

        if value <= 3.0:
            return self.msg.max_range

        try:
            distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return self.msg.max_range

        # Convert to meters
        distance /= 100.0

        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = self.msg.max_range
        if distance < self.msg.min_range: distance = self.msg.max_range

        return distance

class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)

    def read_value(self):
        # From the Pololu source code
        milliamps = self.controller.analog_read(self.pin) * 34
        return milliamps / 1000.0

class MotorTotalCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(MotorTotalCurrent, self).__init__(*args, **kwargs)

    def read_value(self):
        mylist=[]
        for i in range(31):
            value = self.controller.analog_read(self.pin)
            mylist.append(value)

        for m in range(30):
            for n in range(0, 30-m):
                if mylist[n] > mylist[n+1]:
                    mylist[n], mylist[n+1] = mylist[n+1], mylist[n]

        midVal = mylist[15]
        result = (midVal/1024.0*4523.00 - 4523.00/2)/100
        return Decimal(result).quantize(Decimal('0.00'))

class VoltageSensor(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(VoltageSensor, self).__init__(*args, **kwargs)

    def read_value(self):
        refVol = 4.77
        voltage = self.controller.analog_read(self.pin)/1024.0*refVol*5
        return round(voltage,2)

class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)

    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.controller.analog_read(self.pin) - 500.)
        return current

class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)

        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']

        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0

    def read_value(self):
        return self.controller.get_MaxEZ1(self.trigger_pin, self.output_pin)


if __name__ == '__main__':
    myController = Controller()
    mySensor = SonarSensor(myController, "My Sonar", type=Type.PING, pin=0, rate=10)

