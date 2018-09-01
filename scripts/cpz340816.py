#! /usr/bin/env python3

import sys
import time
import pyinterface

import rospy
import std_msgs
from std_msgs.msg import Float64


ch_number = 16
node_name = 'cpz340816'


class cpz340816_controller(object):

    def __init__(self):
        self.rsw_id = rospy.get_param('~rsw_id')
        self.flag = 1
        self.ch = 1
        self.voltage = 0.
        self.da = pyinterface.open(3408, self.rsw_id)
        self.topic_list = [rospy.get_param('~topic{}'.format(i+1)) for i in range(ch_number)]
        self.pub_list = [rospy.Publisher(topic, Float64, queue_size=1)
                         for topic in self.topic_list]
        self.sub_list = [rospy.Subscriber(topic+'_cmd', Float64, self.output_voltage, callback_args=ch)
                         for ch, topic in enumerate(self.topic_list, start=1)]

    def set_param(self, req, ch):
        self.ch = ch
        self.voltage = req.data
        self.flag = 0
        pass

    def output_voltage(self):
        while not rospy.is_shutdown():

            if self.flag == 1:
                time.sleep(0.01)
                continue

            da.output_voltage(self.ch, self.voltage)
            self.pub_list[self.ch-1].Publish(self.voltage)
            self.flag = 1
            continue

        
if __name__ == '__main__':
    rospy.init_node(node_name)
    cpz340816_controller()
    rospy.spin()
