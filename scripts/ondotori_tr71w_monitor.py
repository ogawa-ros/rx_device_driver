#! /usr/bin/env python3

# import modules
import rospy
from std_msgs.msg import Float64
import sys

import time
import urllib.request

node_name = 'tr71w'

class tr71w(object):
    def __init__(self, IP='192.168.100.1'):
        self.IP = IP
        self.url = 'http://'+self.IP+'/B/crrntdata/cdata.txt'

    def temp(self):
        res = urllib.request.urlopen(self.url)
        page = res.read()
        decoded_page = page.decode('shift_jis')
        raw_data = decoded_page.split('\r\n')
        raw_T1 = raw_data[5].split('=')
        raw_T2 = raw_data[6].split('=')
        if raw_T1[1] != '----':
            temp1 = float(raw_T1[1])
        else:
            temp1 = 0
        if raw_T2[1] != '----':
            temp2 = float(raw_T2[1])
        else:
            temp2 = 0
        return temp1, temp2


if __name__ == '__main__':
    rospy.init_node(node_name)

    host = rospy.get_param('~host')
    rate = rospy.get_param('~rate')
    ondotori_no = rospy.get_param('~ondotori_no')

    name1 = node_name + ondotori_no + '_temp_ch1'
    name2 = node_name + ondotori_no + '_temp_ch2'

    try:
        ondo = tr71w(host)
    except OSError as e:
        rospy.logerr("{e.strerror}. host={host}".format(**locals()))
        sys.exit()

    pub_temp1 = rospy.Publisher(name1, Float64, queue_size=1)
    pub_temp2 = rospy.Publisher(name2, Float64, queue_size=1)

    while not rospy.is_shutdown():

        ret = ondo.temp()
        msg_temp1 = Float64()
        msg_temp2 = Float64()
        msg_temp1.data = float(ret[0])
        msg_temp2.data = float(ret[1])

        pub_temp.publish(msg_temp1)
        pub_temp.publish(msg_temp2)

        continue
