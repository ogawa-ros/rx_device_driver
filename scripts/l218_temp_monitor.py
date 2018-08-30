#! /usr/bin/env python3

import sys
import time
import threading
import pymeasure

import rospy
import std_msgs
from std_msgs.msg import Float64


class lakeshore218_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)

    def measure(self):
        self.com.open()
        self.com.send('KRDG?')
        raw = self.com.readline()
        ret = raw.strip().split(',')
        self.com.close()
        temperature = list(map(float, ret))
        
        return temperature

    def measure_ch(self, ch=1):
        self.com.open()
        self.com.send('KRDG? %d'%(ch))
        ret = self.com.readline()
        self.com.close()
        temperature = float(ret)

        return temperature

def str2list(param):
    return param.strip('[').strip(']').split(',')
    
if __name__ == '__main__':
    node_name = 'lakeshore_218'
    rospy.init_node(node_name)

    host = rospy.get_param('~host')
    port = rospy.get_param('~port')
    rate = rospy.get_param('~rate')

    topic_list = [str2list(rospy.get_param('~topic_name{}'.format(i+1))) for i in range(8)]
    '''
    topic_name1 = str2list(rospy.get_param('~topic_name1'))
    topic_name2 = str2list(rospy.get_param('~topic_name2'))
    topic_name3 = str2list(rospy.get_param('~topic_name3'))
    topic_name4 = str2list(rospy.get_param('~topic_name4'))
    topic_name5 = str2list(rospy.get_param('~topic_name5'))
    topic_name6 = str2list(rospy.get_param('~topic_name6'))
    topic_name7 = str2list(rospy.get_param('~topic_name7'))
    topic_name8 = str2list(rospy.get_param('~topic_name8'))
    '''

    try:
        temp = lakeshore218_driver(host, port)
    except OSError as e:
        rospy.logerr("{e.strerror}. host={host}".format(**locals()))
        sys.exit()

    pub_list = [rospy.Publisher(i[0], Float64, queue_size=1) for i in topic_list]
    '''
    pub1 = rospy.Publisher(topic_name1[0], Float64, queue_size=1)
    pub2 = rospy.Publisher(topic_name2[0], Float64, queue_size=1)
    pub3 = rospy.Publisher(topic_name3[0], Float64, queue_size=1)
    pub4 = rospy.Publisher(topic_name4[0], Float64, queue_size=1)
    pub5 = rospy.Publisher(topic_name5[0], Float64, queue_size=1)
    pub6 = rospy.Publisher(topic_name6[0], Float64, queue_size=1)
    pub7 = rospy.Publisher(topic_name7[0], Float64, queue_size=1)
    pub8 = rospy.Publisher(topic_name8[0], Float64, queue_size=1)
    '''
    msg_list = [Float64() for i in range(8)]
    
    while not rospy.is_shutdown():

        ret = temp.measure()

        for h, i, j, k in zip(topic_list, pub_list, msg_list, range(8)):
            if int(h[1]) == 0: pass
            elif int(h[1]) == 1:
                j.data = ret[k]
                i.publish(j)
                     
        '''
        if int(topic_name1[1]) == 0: pass
        elif int(topic_name1[1]) == 1:
            msg1 = Float64()
            msg1.data = ret[0]
            # msg1.data = temp.measure_ch(1)
            pub1.publish(msg1)
        
        if int(topic_name2[1]) == 0: pass
        elif int(topic_name2[1]) == 1:
            msg2 = Float64()
            msg2.data = ret[1]
            # msg2.data = temp.measure_ch(2)
            pub2.publish(msg2)

        if int(topic_name3[1]) == 0: pass
        elif int(topic_name3[1]) == 1:
            msg3 = Float64()
            msg3.data = ret[2]           
            # msg3.data = temp.measure_ch(3)
            pub3.publish(msg3)

        if int(topic_name4[1]) == 0: pass
        elif int(topic_name4[1]) == 1:
            msg4 = Float64()
            msg4.data = ret[3]                       
            # msg4.data = temp.measure_ch(4)
            pub4.publish(msg4)

        if int(topic_name5[1]) == 0: pass
        elif int(topic_name5[1]) == 1:
            msg5 = Float64()
            msg5.data = ret[4]                       
            # msg5.data = temp.measure_ch(5)
            pub5.publish(msg5)

        if int(topic_name6[1]) == 0: pass
        elif int(topic_name6[1]) == 1:
            msg6 = Float64()
            msg6.data = ret[5]                       
            # msg6.data = temp.measure_ch(6)
            pub6.publish(msg6)
        
        if int(topic_name7[1]) == 0: pass
        elif int(topic_name1[1]) == 1:
            msg7 = Float64()
            msg7.data = ret[7]                       
            # msg7.data = temp.measure_ch(7)
            pub7.publish(msg7)

        if int(topic_name8[1]) == 0: pass
        elif int(topic_name8[1]) == 1:
            msg8 = Float64()
            msg8.data = ret[7]                       
            # msg8.data = temp.measure_ch(8)
            pub8.publish(msg8)
        '''
