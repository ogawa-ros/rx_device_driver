#! /usr/bin/env python3

import sys
import time
import socket

import rospy
import std_msgs
from std_msgs.msg import Float64


class tpg261_driver(object):
    
    def __init__(self, IP='', port=9600):
        self.IP = IP
        self.port = port
        self.clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientsock.connect((self.IP, self.port))

    def query_pressure(self):
        self.clientsock.sendall(bytes('PR1 \r\n', 'utf8'))
        time.sleep(0.5)
        self.clientsock.sendall(bytes('\x05', 'utf8'))
        time.sleep(0.5)
        raw = self.clientsock.recv(256)
        raw_dec = raw.decode('utf8')
        line = raw_dec.split('\r\n')
        ret = line[1].split(',')
        pressure = float(ret[1])

        return pressure

if __name__ == '__main__':
    node_name = 'lakeshore_218'
    rospy.init_node(node_name)

    ch_number = 8
    topic_name_index = 0
    onoff_index = 1
    host = rospy.get_param('~host')
    port = rospy.get_param('~port')
    rate = rospy.get_param('~rate')
    topic = rospy.get_param('~topic')

    try:
        pressure = tpg261_driver(host, port)
    except OSError as e:
        rospy.logerr("{e.strerror}. host={host}".format(**locals()))
        sys.exit()

        pub = rospy.Publisher(topic, Float64, queue_size=1)
        msg = Float64()

    while not rospy.is_shutdown():

        ret = pressure.measure()
        msg.data = ret
        pub.publish(msg)
        continue
