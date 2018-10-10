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
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.axis = rospy.get_param('~axis')
        self.node_name = rospy.get_param('~node_name')
        self.jog_flag = False
        self.ptp_flag = False
        self.pulse_num_cmd_flag = False
        self.pulse_num_flag = False
        self.pulse_num_cmd_li = []
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
        ###=== Define topic ===###
        topic_jog_onoff_cmd = '/{0}_rsw{1}_{2}_jog_onoff_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_ptp_onoff_cmd = '/{0}_rsw{1}_{2}_ptp_onoff_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_pulse_num_cmd = '/{0}_rsw{1}_{2}_pulse_num_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_pulse_num = '/{0}_rsw{1}_{2}_pulse_num'.format(self.node_name, self.rsw_id, self.axis)
        topic_onoff = '/{0}_rsw{1}_{2}_onoff'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        self.pub_pulse_num = rospy.Publisher(topic_pulse_num, Int64, queue_size=1)
        self.pub_onoff = rospy.Publisher(topic_onoff, Bool, queue_size=1)
        ###=== Define Subscriber ===###
        self.sub_jog_switch = rospy.Subscriber(topic_jog_onoff_cmd, Bool, self.jog_switch)
        self.sub_ptp_switch = rospy.Subscriber(topic_ptp_onoff_cmd, Bool, self.ptp_switch)
        self.sub_pulse_num_cmd = rospy.Subscriber(topic_pulse_num_cmd, Int64, self.pulse_num_cmd_switch)
        self.sub_pulse_num = rospy.Subscriber(topic_pulse_num, Bool, self.pulse_num_switch)

    def jog_switch(self, q):
        self.jog_flag = q.data
        return

    def ptp_switch(self, q):
        self.ptp_flag = q.data
        return

    def pulse_num_cmd_switch(self, q):
        self.pulse_num_cmd_li.append(q.data)
        self.pulse_num_cmd_flag = True
        pass

    def pulse_num_switch(self, q):
        self.pulse_num_flag = q.data
        return

    def move_jog(self):
        while not rospy.is_shutdown():
            ###=== Standby loop without pulse output ===###
            if self.jog_flag == False:
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
            while self.jog_flag == bool(self.mot.check_move_onoff(axis=self.axis)[0]):
                time.sleep(self.rate)
                continue
            ###=== End of JOG operation ===###
            self.mot.stop(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            continue

    def move_ptp(self):
        while not rospy.is_shutdown():
            ###=== Standby loop without pulse output ===###
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
            ###=== Standby loop with pulse output ===###
            while self.ptp_flag == bool(self.mot.check_move_onoff(axis=self.axis)[0]):
                time.sleep(self.rate)
                self.ptp_flag = False
                continue

    def set_pulse_num(self):
        while not rospy.is_shutdown():
            ###=== Standby loop for set pulse_num ===###
            if self.pulse_num_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set pulse_num ===###
            self.mot.set_pulse_num(axis=self.axis, pls_num=self.pulse_nun_cmd_li)
            self.pulse_num_flag = True
            self.pulse_num_cmd_li = []
            self.pulse_num_cmd_flag = False
            continue

    def get_pulse_num(self):
        while not rospy.is_shutdown():
            ###=== Standby loop for get pulse_num ===###
            if self.pulse_num_flag == False:
                time.sleep(self.rate)
                continue
            ###=== publish pulse_num ===###
            pulse_num = self.mot.get_pulse_num(axis=self.axis)[0]
            self.pub_pulse_num.publish(pulse_num)
            self.pulse_num_flag = False
            continue

    def check_move_onoff(self):
        self.pub_onoff.publish(self.mot.check_move_onoff(axis=self.axis)[0])
        while not rospy.is_shutdown():
            ###=== Standby loop without pulse output ===###
            if bool(self.mot.check_move_onoff(axis=self.axis)[0]) == False:
                time.sleep(self.rate)
                continue
            ###=== publish onoff ===###
            self.pub_onoff.publish(bool(self.mot.check_move_onoff(axis=self.axis)[0]))
            ###=== Standby loop with pulse output ===###
            while bool(self.mot.check_move_onoff(axis=self.axis)[0]) == True:
                time.sleep(self.rate)
                continue
            ###=== publish onoff ===###
            self.pub_onoff.publish(bool(self.mot.check_move_onoff(axis=self.axis)[0]))
            continue

    def start_thread_ROS(self):
        th1 = threading.Thread(target=self.move_jog)
        th2 = threading.Thread(target=self.move_ptp)
        th3 = threading.Thread(target=self.set_pulse_num)
        th4 = threading.Thread(target=self.get_pulse_num)
        th5 = threading.Thread(target=self.check_move_onoff)
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th4.setDaemon(True)
        th5.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()
        th4.start()
        th5.start()
        return

if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
