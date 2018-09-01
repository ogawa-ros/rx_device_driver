#! /usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from rx_device_driver.msg import e8250d_msg

import sys
import time
import pymeasure

node_name = 'e8250d'


class e8257d_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        
    def set_freq(self, freq, unit='GHz'):
        self.com.open()
        self.com.send('FREQ:CW %.10f %s'%(freq, unit))
        self.com.close()
        return

    def get_freq(self):
        self.com.open()
        self.com.send('FREQ:CW?')
        ret = self.com.readline()
        self.com.close()

        return float(ret)/1e+9

    def set_power(self, power=-20.0):
        self.com.open()
        self.com.send('POW %f dBm'%(power))
        self.com.close()
        return

    def get_power(self):
        self.com.open()
        self.com.send('POW?')
        ret = self.com.readline()
        self.com.close()

        return float(ret)

    def set_onoff(self, onoff=0):
        self.com.open()
        if onoff==1:
            self.com.send('OUTP ON')
        else:
            self.com.send('OUTP OFF')
        self.com.close()
        return

    def get_onoff(self):
        self.com.open()
        self.com.send('OUTP?')
        ret = self.com.readline()
        self.com.close()

        return int(ret)


class e8257d_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        self.sg = e8257d_driver(host, port)
        
        topic_pub_freq = rospy.get_param('~topic_pub_freq')
        topic_pub_power = rospy.get_param('~topic_pub_power')
        topic_pub_onoff = rospy.get_param('~topic_pub_onoff')
        topic_pub_status = rospy.get_param('~topic_pub_status')
        topic_sub_freq = rospy.get_param('~topic_sub_freq')
        topic_sub_power = rospy.get_param('~topic_sub_power')
        topic_sub_onoff = rospy.get_param('~topic_sub_onoff')
        topic_sub_status = rospy.get_param('~topic_sub_status')

        self.pub_freq = rospy.Publisher('{}'.format(topic_pub_freq), Float64, queue_size=1)
        self.pub_power = rospy.Publisher('{}'.format(topic_pub_power), Float64, queue_size=1)
        self.pub_onoff = rospy.Publisher('{}'.format(topic_pub_onoff), Int32, queue_size=1)
        self.pub_status = rospy.Publisher('{}'.format(topic_pub_status), e8250d_msg, queue_size=1)
        self.sub_freq = rospy.Subscriber('{}'.format(topic_sub_freq), Float64, self.set_freq)
        self.sub_power = rospy.Subscriber('{}'.format(topic_sub_power), Float64, self.set_power)
        self.sub_onoff = rospy.Subscriber('{}'.format(topic_sub_onoff), Int32, self.set_onoff)
        self.sub_status = rospy.Subscriber('{}'.format(topic_sub_status), Int32, self.get_status)
        
    def set_freq(self, q):
        target = q.data
        self.sg.set_freq(target, 'GHz')
        self.pub_freq.publish(self.sg.get_freq())

    def set_power(self, q):
        target = q.data
        self.sg.set_power(target)
        self.pub_power.publish(self.sg.get_power())

    def set_onoff(self, q):
        target = q.data
        self.sg.set_onoff(target)
        self.pub_onoff.publish(self.sg.get_onoff())

    def get_status(self, q):
        status = e8250d_msg()
        status.freq = self.sg.get_freq()
        status.power = self.sg.get_power()
        status.onoff = self.sg.get_onoff()
        
        self.pub_status.publish(status)
        return


if __name__ == '__main__':
    rospy.init_node(node_name)
    e8257d_controller()
    rospy.spin()
