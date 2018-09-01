#! /usr/bin/env python3

import sys
import time
import threading
import pyinterface

import rospy
import std_msgs
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String


ch_number = 8
node_name = 'cpz340516'


class cpz340516_controller(object):

    def __init__(self):
        self.rsw_id = rospy.get_param('~rsw_id')
        self.flag = 1
        self.flag_ouputrange = 1
        self.flag_onoff = 1
        self.ch = 1
        self.current = 0.
        self.ouputrange = 'DA0_100mA'
        self.onoff = 0
        self.da = pyinterface.open(3405, self.rsw_id)
        self.topic_list = [rospy.get_param('~topic{}'.format(i+1)) for i in range(ch_number)]
        self.pub_list = [rospy.Publisher(topic, Float64, queue_size=1)
                         for topic in self.topic_list]
        self.sub_list = [rospy.Subscriber(topic+'_cmd', Float64, self.output_current, callback_args=ch)
                         for ch, topic in enumerate(self.topic_list, start=1)]
        self.topic_outputrange_list = [rospy.get_param('~topic_outputrange{}'.format(i+1)) for i in range(ch_number)]
        self.pub_outputrange_list = [rospy.Publisher(topic, String, queue_size=1)
                                     for topic in self.topic_outputrange_list]
        self.sub_outputrange_list = [rospy.Subscriber(topic+'_cmd', String, self.set_outputrange, callback_args=ch)
                                     for ch, topic in enumerate(self.topic_outputrange_list, start=1)]
        self.topic_onoff_list = [rospy.get_param('~topic_onoff{}'.format(i+1)) for i in range(ch_number)]
        self.pub_onoff_list = [rospy.Publisher(topic, Int32, queue_size=1)
                                     for topic in self.topic_onoff_list]
        self.sub_onoff_list = [rospy.Subscriber(topic+'_cmd', Int32, self.set_onoff, callback_args=ch)
                                     for ch, topic in enumerate(self.topic_onoff_list, start=1)]
        self.outputrange_status = [rospy.Subscriber(topic+'_stat', String, self.get_outputrange, callback_args=ch)
                                   for ch, topic in enumerate(self.topic_outputrange_list, start=1)]
        self.onoff_status = [rospy.Subscriber(topic+'_stat', Int32, self.get_onoff, callback_args=ch)
                                 for ch, topic in enumerate(self.topic_onoff_list, start=1)]

        
    def set_current_param(self, req, ch):
        self.ch = ch
        self.current = req.data
        self.flag = 0
        pass            

    def set_outputrange_param(self, req, ch):
        self.ch = ch
        self.outputrange = req.data
        self.flag = 0
        pass            

    def set_onoff_param(self, req, ch):
        self.ch = ch
        self.onoff = req.data
        self.flag = 0
        pass            
    
    def output_current(self):
        while not rospy.is_shutdown():

            if self.flag == 1:
                time.sleep(0.01)
                continue

            da.output_current(self.ch, self.current)
            self.pub_list[self.ch-1].Publish(self.current)
            self.flag = 1
            continue

    def set_outputrange(self):
        while not rospy.is_shutdown():

            if self.flag == 1:
                time.sleep(0.01)
                continue

            da.output_outputrange(self.ch, self.outputrange)
            self.pub_list[self.ch-1].Publish(self.outputrange)
            self.flag = 1
            continue

    def set_onoff(self):
        while not rospy.is_shutdown():

            if self.flag == 1:
                time.sleep(0.01)
                continue

            da.output_onoff(self.ch, self.onoff)
            self.pub_list[self.ch-1].Publish(self.onoff)
            self.flag = 1
            continue

    def get_outputrange(self, req, ch):
        self.ch = ch
        outputrange = da.get_outputrange(self.ch)['ch{}'.format(self.ch)]
        msg = String()
        msg.data = outputrange
        self.pub_outputrange_list[self.ch1].Publish(msg)
        return

    def get_onoff(self, req, ch):
        self.ch = ch
        onoff = da.get_onoff(self.ch)['ch{}'.format(self.ch)]
        msg = Int32()
        msg.data = onoff
        self.pub_onoff_list[self.ch].Publish(msg)
        return

    def start_thread_ROS(self):
        th = threading.Thread(target=self.output_current)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target=self.set_outputrange)
        th2.setDaemon(True)
        th2.start()        
        th3 = threading.Thread(target=self.set_onoff)
        th3.setDaemon(True)
        th3.start()        

if __name__ == '__main__':
    rospy.init_node(node_name)
    ctrl = cpz340516_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
