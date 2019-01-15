#! /usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import sys
import time
import pymeasure


node_name = 'e8257d'


class InvalidRangeError(Exception):
    pass


class e8257d_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB

    def set_freq(self, freq, unit='GHz'):
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('FREQ:CW %.10f %s'%(freq, unit))
        self.com.close()
        return

    def get_freq(self):
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('FREQ:CW?')
        ret = self.com.readline()
        self.com.close()

        return float(ret)/1e+9

    def set_power(self, power=-20.0):
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
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
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('POW?')
        ret = self.com.readline()
        self.com.close()

        return float(ret)

    def set_onoff(self, onoff=0):
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        if onoff==1:
            self.com.send('OUTP ON')
        else:
            self.com.send('OUTP OFF')
        self.com.close()
        return

    def get_onoff(self):
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('OUTP?')
        ret = self.com.readline()
        self.com.close()

        return int(ret)


class e8257d_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        self.sg_name = rospy.get_param('~sg_name')
        self.sg = e8257d_driver(host, port)

        topic_list = ['freq', 'power', 'onoff']
        data_class_list = [Float64, Float64, Int32]
        pub_list = [
            rospy.Publisher(
                name = '{0}_{1}'.format(self.sg_name, topic),
                data_class = _data_class,
                latch = True,
                queue_size = 1
            )
            for topic, _data_class in zip(topic_list, data_class_list)
        ]
        sub_list = [
            rospy.Publisher(
                name = '{0}_{1}_cmd'.format(self.sg_name, topic),
                data_class = _data_class,
                callback = self.callback
                callback_args = topic
                queue_size = 1
            )
            for topic, _data_class in zip(topic_list, data_class_list)
        ]

    def callback(self, q, topic):
        target = q.data
        exec('self.sg.set_{}(target)'.format(topic))
        current = exec('self.sg.get_{}()'.format(topic))
        self.pub_list[topic_list.index(topic)].publish(current)
        return


if __name__ == '__main__':
    rospy.init_node(node_name)
    e8257d_controller()
    rospy.spin()
