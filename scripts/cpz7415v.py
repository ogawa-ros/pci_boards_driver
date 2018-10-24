#! /usr/bin/env python3

import sys
import time
import numpy
import pyinterface
import threading

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool

class cpz7415v_controller(object):

    def __init__(self):
        ###=== Define member-variables ===###
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.axis = rospy.get_param('~axis')
        self.node_name = rospy.get_param('~node_name')
        self.mode = rospy.get_param('~mode')
        self.position_cmd_flag = False
        self.speed_cmd_flag = False
        self.busy_flag = False
        self.position_cmd_li = []
        self.speed_cmd_li = []
        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        ###=== Setting the board ===###
        self.mot.initialize(axis=self.axis)
        self.mot.driver.pMotion[self.mode]['clock'][self.axis] = rospy.get_param('~clock')
        self.mot.driver.pMotion[self.mode]['position'][self.axis] = rospy.get_param('~position')
        self.mot.driver.pMotion[self.mode]['fl_speed'][self.axis] = rospy.get_param('~fl_speed')
        self.mot.driver.pMotion[self.mode]['fh_speed'][self.axis] = rospy.get_param('~fh_speed')
        self.mot.driver.pMotion[self.mode]['acc_rate'][self.axis] = rospy.get_param('~acc_rate')
        self.mot.driver.pMotion[self.mode]['dec_rate'][self.axis] = rospy.get_param('~dec_rate')
        self.mot.set_motion(axis=self.axis, mode=self.mode)
        ###=== Define topic ===###
        topic_position_cmd = '/{0}_rsw{1}_{2}_position_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_position = '/{0}_rsw{1}_{2}_position'.format(self.node_name, self.rsw_id, self.axis)
        topic_speed_cmd = '/{0}_rsw{1}_{2}_speed_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_speed = '/{0}_rsw{1}_{2}_speed'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        self.pub_position = rospy.Publisher(topic_position, Int64, queue_size=1)
        self.pub_speed = rospy.Publisher(topic_speed, Int64, queue_size=1)
        ###=== Define Subscriber ===###
        self.sub_position_cmd = rospy.Subscriber(topic_position_cmd, Int64, self.position_cmd_switch)
        self.sub_speed_cmd = rospy.Subscriber(topic_speed_cmd, Int64, self.speed_cmd_switch)


    def position_cmd_switch(self, q):
        self.position_cmd_li.append(q.data)
        self.position_cmd_flag = True
        return


    def position_switch(self, q):
        self.position_flag = q.data
        return


    def speed_cmd_switch(self, q):
        self.speed_cmd_li.append(q.data)
        self.speed_cmd_flag = True
        return


    def speed_switch(self, q):
        self.speed_flag = True
        return


    def set_position(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set position ===###
            if self.position_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set position ===###
            if self.busy_flag == False:
                self.busy_flag = True
                self.mot.driver.pMotion[self.mode]['position'][self.axis] = self.position_cmd_li[0]
                self.mot.set_motion(axis=self.axis, mode='PTP')
                time.sleep(self.rate)
                self.mot.start_motion(axis=self.axis, stamod='staud', movmod='PTP')
                self.position_cmd_li = []
                self.position_cmd_flag = False
                self.busy_flag = False
            else: pass
            continue


    def get_position(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get position ===###
            if last_position == self.mot.read_counter(axis=self.axis)[0]:
                last_position = self.mot.read_counter(axis=self.axis)[0]
                time.sleep(self.rate)
                continue
            ###=== publish position ===###
            if self.busy_flag == False:
                self.busy_flag = True
                position = self.mot.read_counter(axis=self.axis)[0]
                self.pub_position.publish(position)
                last_position = position
                time.sleep(self.rate)
                self.busy_flag = False
            else: pass
            continue


    def set_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set speed ===###
            if self.speed_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set speed ===###
            if self.busy_flag == False:
                self.busy_flag = True
                self.mot.change_speed(axis=self.axis, spd=self.speed_cmd_li)
                self.speed_cmd_li = []
                self.speed_cmd_flag = False
                self.busy_flag = False
            else: pass
            continue


    def get_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get speed ===###
            if last_speed == self.mot.read_speed(axis=self.axis)[0]:
                last_speed = self.mot.read_speed(axis=self.axis)[0]
                time.sleep(self.rate)
                continue
            ###=== publish speed ===###
            if self.busy_flag == False:
                self.busy_flag = True
                speed = self.mot.read_speed(axis=self.axis)[0]
                self.pub_speed.publish(speed)
                last_speed = speed
                time.sleep(self.rate)
                self.busy_flag = False
            else: pass
            continue


    def start_thread_ROS(self):
        th1 = threading.Thread(target=self.set_position)
        th2 = threading.Thread(target=self.get_position)
        th3 = threading.Thread(target=self.set_speed)
        th4 = threading.Thread(target=self.get_speed)
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th4.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()
        th4.start()
        return


if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    ctrl.mot.start_motion(axis=ctrl.axis, stamod='staud', movmod=ctrl.mode)
    rospy.spin()
