#! /usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import sys
import time
import pymeasure


node_name = 'mg3692c'


class InvalidRangeError(Exception):
    pass


class mg3692c_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        # self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com = pymeasure.ethernet(self.IP, self.GPIB)

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
        freq = float(ret) / 1e+9

        return freq

    def set_power(self, power=-20.0):
        if -20.0<=power<=30.0:
            self.com.open()
            self.com.send('POW %f dBm'%(power))
            self.com.close()
        else:
            msg = 'Power range is -20.0[dBm] -- 30.0[dBm],'
            msg += ' while {}[dBm] is given.'.format(power)
            raise InvalidRangeError(msg)
        return

    def get_power(self):
        self.com.open()
        self.com.send('POW?')
        ret = self.com.readline()
        self.com.close()
        power = float(ret)

        return power

    def set_onoff(self, onoff=0):
        self.com.open()
        if onoff==1:
            self.com.send('OUTP ON')
            self.com.close()
        else:
            self.com.send('OUTP OFF')
            self.com.close()
        return

    def get_onoff(self):
        self.com.open()
        self.com.send('OUTP?')
        ret = self.com.readline()
        self.com.close()
        ret = int(ret)

        return ret


class mg3692c_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        name = rospy.get_param('~node_name')
        self.sg = mg3692c_driver(host, port)

        self.pub_freq = rospy.Publisher(
            name = '{}_freq'.format(name),
            data_class = Float64,
            latch = True,
            queue_size = 1
            )
        self.pub_power = rospy.Publisher(
            name = '{}_power'.format(name),
            data_class = Float64,
            latch = True,
            queue_size = 1
            )
        self.pub_freq = rospy.Publisher(
            name = '{}_onoff'.format(name),
            data_class = Int32,
            latch = True,
            queue_size = 1
            )

        self.sub_freq = rospy.Subscriber(
            name = '{}_freq_cmd'.format(name),
            data_class = Float64,
            callback = self.callback_freq,
            queue_size = 1
            )
        self.sub_power = rospy.Subscriber(
            name = '{}_power_cmd'.format(name),
            data_class = Float64,
            callback = self.callback_power,
            queue_size = 1
            )
        self.sub_onoff = rospy.Subscriber(
            name = '{}_onoff_cmd'.format(name),
            data_class = Int32,
            callback = self.callback_onoff,
            queue_size = 1
            )

    def callback_freq(self, q):
        target = q.data
        self.sg.set_freq(target)
        time.sleep(1)
        current = self.sg.get_freq()
        self.pub_freq.publish(current)
        return

    def callback_power(self, q):
        target = q.data
        self.sg.set_power(target)
        time.sleep(1)
        current = self.sg.get_power()
        self.pub_power.publish(current)
        return

    def callback_onoff(self, q):
        target = q.data
        self.sg.set_onoff(target)
        time.sleep(1)
        current = self.sg.get_onoff()
        self.pub_onoff.publish(current)
        return


if __name__ == '__main__':
    rospy.init_node(node_name)
    mg3692c_controller()
    rospy.spin()
