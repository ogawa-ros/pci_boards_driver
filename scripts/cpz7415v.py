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
        self.ptp_flag = False
        self.position_cmd_flag = False
        self.speed_cmd_flag = False
        self.move_to_home_flag = False
        self.position_cmd_li = []
        self.speed_cmd_li = []
        self.target_speed = rospy.get_param('~fh_speed')
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
        topic_ptp_onoff_cmd = '/{0}_rsw{1}_{2}_ptp_onoff_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_position_cmd = '/{0}_rsw{1}_{2}_position_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_position = '/{0}_rsw{1}_{2}_position'.format(self.node_name, self.rsw_id, self.axis)
        topic_speed_cmd = '/{0}_rsw{1}_{2}_speed_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_speed = '/{0}_rsw{1}_{2}_speed'.format(self.node_name, self.rsw_id, self.axis)
        topic_move_to_home = '/{0}_rsw{1}_{2}_move_to_home'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        self.pub_position = rospy.Publisher(topic_position, Int64, queue_size=1)
        self.pub_speed = rospy.Publisher(topic_speed, Int64, queue_size=1)
        ###=== Define Subscriber ===###
        self.sub_ptp_switch = rospy.Subscriber(topic_ptp_onoff_cmd, Bool, self.ptp_switch)
        self.sub_position_cmd = rospy.Subscriber(topic_position_cmd, Int64, self.position_cmd_switch)
        self.sub_speed_cmd = rospy.Subscriber(topic_speed_cmd, Int64, self.speed_cmd_switch)
        self.sub_position = rospy.Subscriber(topic_position, Int64, self.position_switch)
        self.sub_move_to_home = rospy.Subscriber(topic_move_to_home, Bool, self.move_to_home_switch)

    def ptp_switch(self, q):
        self.ptp_flag = q.data
        return

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

    def move_to_home_switch(self, q):
        self.move_to_home_flag = q.data
        return

    def move_ptp(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop without pulse output ===###
            if self.ptp_flag == False:
                time.sleep(self.rate)
                continue
            ###=== Start pulse output ===###
            if self.mot.move_mode[self.axis] == 'PTP': pass
            else:
                self.mot.set_mode(mode=['PTP'], axis=self.axis)
                time.sleep(self.rate)
            self.mot.move(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            ###=== Stand-by loop with pulse output ===###
            while self.ptp_flag == bool(self.mot.check_move_onoff(axis=self.axis)[0]):
                self.ptp_flag = False
                time.sleep(self.rate)
                continue

    def set_position(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set position ===###
            if self.position_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set position ===###
            self.mot.driver.pMotion[self.mode]['position'][self.axis] = self.position_cmd_li[0]
            self.mot.set_motion(axis=self.axis, mode='PTP')
            time.sleep(self.rate)
            self.mot.start_motion(axis=self.axis, stamod='staud', movmod='PTP')
            self.position_cmd_li = []
            self.position_cmd_flag = False
            continue

    def get_position(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get position ===###
            if self.mot.driver.pMotion[self.mode]['position'][self.axis] == self.mot.read_counter(axis=self.axis)[0]:
                time.sleep(self.rate)
                continue
            ###=== publish position ===###
            position = self.mot.read_counter(axis=self.axis)[0]
            self.pub_position.publish(position)
            time.sleep(self.rate)
            continue

    def set_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set speed ===###
            if self.speed_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set speed ===###
            self.mot.change_speed(axis=self.axis, spd=self.speed_cmd_li)
            self.target_spd = self.speed_cmd_li[0]
            self.speed_cmd_li = []
            self.speed_cmd_flag = False
            continue

    def get_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get speed ===###
            if self.target_speed == self.mot.read_speed(axis=self.axis)[0]:
                time.sleep(self.rate)
                continue
            ###=== publish speed ===###
            speed = self.mot.read_speed(axis=self.axis)[0]
            self.pub_speed.publish(speed)
            time.sleep(0.1)
            continue

    def move_to_home(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for move to home ===###
            if self.move_to_home_flag == False:
                time.sleep(self.rate)
                continue
            ###=== move to home ===###
            self.mot.move_to_home(axis=self.axis)
            time.sleep(self.rate)
            self.move_to_home_flag = False
            continue

    def start_thread_ROS(self):
        th1 = threading.Thread(target=self.move_ptp)
        th2 = threading.Thread(target=self.set_position)
        th3 = threading.Thread(target=self.get_position)
        th4 = threading.Thread(target=self.set_speed)
        th5 = threading.Thread(target=self.get_speed)
        th6 = threading.Thread(target=self.move_to_home)
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th4.setDaemon(True)
        th5.setDaemon(True)
        th6.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()
        th4.start()
        th5.start()
        th6.start()
        return

if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    ctrl.mot.start_motion(axis=ctrl.axis, stamod='staud', movmod=ctrl.mode)
    rospy.spin()
