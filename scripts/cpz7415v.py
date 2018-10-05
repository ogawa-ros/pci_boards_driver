#! /usr/bin/env python3

import sys
import time
import numpy
import pyinterface
import threading

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64

class cpz7415v_controller(object):

    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.axis = rospy.get_param('~axis')
        self.node_name = rospy.get_param('~node_name')
        self.jog_flag = 0
        self.ptp_flag = 0
        self.length_flag = 0
        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        ###=== Initialize the board ===###
        self.mot.initializer(axis=self.axis, mode=['JOG'])
        if self.mot.move_mode[self.axis] == 'JOG': pass
        else: self.mot.move_to_home(axis=self.axis)
        ###=== Define topic ===
        topic_jog_switch = '{0}_rsw{1}_{2}_jog_switch'.format(self.node_name, self.rsw_id, self.axis)
        topic_ptp_switch = '{0}_rsw{1}_{2}_ptp_switch'.format(self.node_name, self.rsw_id, self.axis)
        topic_length = '{0}_rsw{1}_{2}_length'.format(self.node_name, self.rsw_id, self.axis)
        #topic_speed = '{0}_rsw{1}_{2}_speed'.format(self.node_name, self.rsw_id, self.axis)
        #topic_position =  '{0}_rsw{1}_{2}_position'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        #self.pub_switch = rospy.Publisher(topic_switch, Int64, queue_size=1)
        #self.pub_speed = rospy.Publisher(topic_speed, Float64, queue_size=1)
        #self.pub_position = rospy.Publisher(topic_position, Float64, queue_size=1)
        ###=== Define Subscriber ===###
        self.jog_switch_sub = rospy.Subscriber(topic_jog_switch + '_cmd', Int64, self.jog_switch)
        self.ptp_switch_sub = rospy.Subscriber(topic_ptp_switch + '_cmd', Int64, self.ptp_switch)
        self.length_sub = rospy.Subscriber(topic_length + '_cmd', Float64, self.set_length)
        #self.sub_speed = rospy.Subscriber(topic_speed + '_cmd', Float64, self.set_speed)
        #self.sub_position = rospy.Publisher(topic_position + '_cmd', Float64, self.set_position)

    def jog_switch(self, q):
        self.jog_flag = q.data
        return

    def ptp_switch(self, q):
        self.ptp_flag = q.data
        return

    def set_length(self, length):
        # temp
        length_li = []
        self.length = length_li.append(length)
        self.length_flag = 1
        pass

    def move_jog(self):
        while not rospy.is_shutdown():
            ###=== Standby loop without pulse output ===###
            if self.jog_flag == 0:
                time.sleep(self.rate)
                continue
            ###=== Start pulse output ===###
            if self.mot.move_mode[self.axis] == 'JOG': pass
            else:
                self.mot.set_mode(mode=['JOG'], axis=self.axis)
                time.sleep(self.rate)
            self.mot.move(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            ###=== Standby loop with pulse output ===###
            while self.jog_flag == self.mot.check_move_onoff(axis=self.axis)[0]:
                time.sleep(self.rate)
                continue
            ###=== End of JOG operation ===###
            self.mot.stop(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            continue

    def move_ptp(self):
        while not rospy.is_shutdown():
            ###=== Standby loop without pulse output ===###
            if self.ptp_flag == 0:
                time.sleep(self.rate)
                continue
            ###=== Start pulse output ===###
            if self.mot.move_mode[self.axis] == 'PTP': pass
            else:
                self.mot.set_mode(mode=['PTP'], axis=self.axis)
                time.sleep(self.rate)
            self.mot.move(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            ###=== Standby loop with pulse output ===###
            while self.ptp_flag == self.mot.check_move_onoff(axis=self.axis)[0]:
                time.sleep(self.rate)
                self.ptp_flag = 0
                continue

    def output_length(self):
        while not rospy.is_shutdown():
            ###=== Standby loop for set length ===###
            if self.length_flag == 0:
                time.sleep(self.rate)
                continue
            ###=== set length ===###
            self.mot.set_length(axis=self.axis, length=self.length)
            self.length_flag == 0
            continue

    def start_thread_ROS(self):
        th1 = threading.Thread(target=self.move_jog)
        th2 = threading.Thread(target=self.move_ptp)
        th3 = threading.Thread(target=self.output_length)
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()

if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
