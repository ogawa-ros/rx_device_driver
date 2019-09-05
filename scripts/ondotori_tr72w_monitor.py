#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import sys
import time
import urllib.request

name = 'tr72w'

class tr72w(object):
    def __init__(self, IP='192.168.100.1'):
        self.IP = IP
        self.url = 'http://'+self.IP+'/B/crrntdata/cdata.txt'

    def measure(self):
        res = urllib.request.urlopen(self.url)
        page = res.read()
        decoded_page = page.decode('shift_jis')
        raw_data = decoded_page.split('\r\n')
        raw_T1 = raw_data[5].split('=')
        raw_T2 = raw_data[6].split('=')
        temp = raw_T1
        hum = raw_T2

        return [temp, hum]

if __name__ == '__main__':
    rospy.init_node(name)

    host = rospy.get_param('~host')
    rate = rospy.get_param('~rate')
    ondotori_no = rospy.get_param('~ondotori_no')

    name_temp = node_name + ondotori_no + '_temp'
    name_hum = node_name + ondotori_no + '_hum'

    try:
        ondo = tr72w(host)
    except OSError as e:
        rospy.logerr("{e.strerror}. host={host}".format(**locals()))
        sys.exit()

    pub_temp = rospy.Publisher(name_temp, Float64, queue_size=1)
    pub_hum = rospy.Publisher(name_hum, Float64, queue_size=1)

    while not rospy.is_shutdown():

        ret = ondo.measure()
        msg_temp = Float64()
        msg_hum = Float64()
        msg_temp.data = float(ret[0])
        msg_hum.data = float(ret[1])

        pub_temp.publish(msg_temp)
        pub_hum.publish(msg_hum)

        continue
