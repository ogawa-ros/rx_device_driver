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
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()


    def set_freq(self, freq, unit='GHz'):
        self.com.send('FREQ:CW %.10f %s'%(freq, unit))
        return

    def get_freq(self):
        self.com.send('FREQ:CW?')
        ret = self.com.readline()

        return float(ret) / 1e+9

    def set_power(self, power=-20.0):
        if -20.0<=power<=30.0:
            self.com.send('POW %f dBm'%(power))
        else:
            msg = 'Power range is -20.0[dBm] -- 30.0[dBm],'
            msg += ' while {}[dBm] is given.'.format(power)
            raise InvalidRangeError(msg)
        return

    def get_power(self):
        self.com.send('POW?')
        ret = self.com.readline()

        return float(ret)

    def set_onoff(self, onoff=0):
        if onoff==1:
            self.com.send('OUTP ON')
        else:
            self.com.send('OUTP OFF')
        return

    def get_onoff(self):
        self.com.send('OUTP?')
        ret = self.com.readline()

        return int(ret)


class e8257d_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        self.sg_name = rospy.get_param('~sg_name')
        self.sg = e8257d_driver(host, port)

        self.topic_list = ['freq', 'power', 'onoff']
        self.data_class_list = [Float64, Float64, Int32]
        self.pub_list = [
            rospy.Publisher(
                name = '{0}_{1}'.format(self.sg_name, topic),
                data_class = _data_class,
                latch = True,
                queue_size = 1
            )
            for topic, _data_class in zip(self.topic_list, self.data_class_list)
        ]
        self.sub_list = [
            rospy.Subscriber(
                name = '{0}_{1}_cmd'.format(self.sg_name, topic),
                data_class = _data_class,
                callback = self.callback,
                callback_args = topic,
                queue_size = 1
            )
            for topic, _data_class in zip(self.topic_list, self.data_class_list)
        ]

    def callback(self, q, topic):
        target = q.data
        exec('self.sg.set_{0}(target)'.format(topic))
        time.sleep(0.5)
        current = exec('self.sg.get_{0}()'.format(topic))
        self.pub_list[self.topic_list.index(topic)].publish(current)
        return


if __name__ == '__main__':
    rospy.init_node(node_name)
    e8257d_controller()
    rospy.spin()
