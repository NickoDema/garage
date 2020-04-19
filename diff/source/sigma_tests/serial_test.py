#!/usr/bin/env python

##   serial_test.py
##   Created on: 29.08.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##               ScPA StarLine Ltd


import os
import sys
import serial
import re

sign = lambda a: 1 if a>0 else -1 if a<0 else 0


class serial_comm():
    def __init__(self):
        self.port_name = '/dev/ttyACM0'
        self.port_is_open = False

    #-----------------------------------------------------------------
    def open_port(self):
        try:
            self.port_is_open = False
            print("[MRP]: Trying to open port: " + self.port_name)
            self.port = serial.Serial(  port=self.port_name,
                                        baudrate=115200,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE,
                                        bytesize=serial.EIGHTBITS)
            self.port_is_open = True
            self.port.flushInput()
            self.port.flushOutput()
            return True
        except serial.SerialException:
            print("[MRP]: ERROR | failed to open port")
            return False

    def get_token(self):
        

    #-----------------------------------------------------------------
    def set_cmd(self, vel_l, vel_r):
        if (self.port.isOpen()):
            try:
                self.port.write("<" + vel_l + " " + vel_r + ">\n")
            except:
                self.port.close()
                self.port.open()
    #-----------------------------------------------------------------
    def get_enc_data(self):
        if (self.port.readable()):
            self.enc_data_str = ""
            try:
                while True:
                    in_ch = self.port.read()
                    #print in_ch
                    if in_ch == '\n':
                        break
                    self.enc_data_str += in_ch

                serObj = re.search(r'<(-?[0-9]+)\s(-?[0-9]+)>', self.enc_data_str, re.I)
                if (serObj):
                    self.ticks_l = int(serObj.group(1))
                    self.ticks_r = int(serObj.group(2))
                    return True
                else:
                    print("Received data doesn't belong to encoder's ticks format: ")
                    print self.enc_data_str
                    return False
            except:
                print("[MRP]: Port reading ERROR")
                return False
        else:
            print("[MRP]: Port is empty to reading")
            return False

    #-----------------------------------------------------------------
    def is_open(self):
        return self.port.isOpen()

    def __del__(self):
        self.port_is_open = False
        try:
            self.port.close()
        except AttributeError:
            print("[MRP]: Port closing ERROR, no port was created")


def cmd_cb(data):
    if(mrp_driver.is_open()):
        if (abs(data.linear.x) > 1.0):
            x = 100*sign(data.linear.x)
        else:
            x = data.linear.x*100
        if (abs(data.angular.z) > 1.0):
            z = 100*sign(data.angular.z)
        else:
            z = data.angular.z*100
        mrp_driver.set_cmd(str(int(x)), str(int(z)))
    else:
        print "[MRP]: Can't sent cmd | port is down"
#       rospy.loginfo("I heard")#,"%s" data.data)


def loop(serial):

    while not rospy.is_shutdown():
        if (mrp_driver.get_enc_data()):
            ticks_str = str(mrp_driver.ticks_l)+" "+str(mrp_driver.ticks_r)
            pub_ticks.publish(ticks_str)
        #hello_str = "hello world %s" % rospy.get_time
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    serial = serial_comm()
    if (serial.open_port()):
        print "port opened"
    else:
        print "can't open"

    loop(serial)
