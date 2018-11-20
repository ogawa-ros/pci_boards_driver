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
        self.pls_num = rospy.get_param('~pls_num')
        self.mag = rospy.get_param('~mag')
        self.fl_spd = rospy.get_param('~fl_spd')
        self.fh_spd = rospy.get_param('~fh_spd')
        self.acc_rate = rospy.get_param('~acc_rate')
        self.dec_rate = rospy.get_param('~dec_rate')
        self.jog_flag = False
        self.ptp_flag = False
        self.pulse_num_cmd_flag = False
        self.pulse_num_flag = False
        self.fh_speed_cmd_flag = False
        self.fh_speed_flag = False
        self.move_to_home_flag = False
        self.pulse_num_cmd_li = []
        self.fh_speed_cmd_li = []
        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        ###=== Initialize the board ===###
        self.mot.initializer(axis=self.axis)
        self.mot.set_mode(mode=[self.mode], axis=self.axis)
        self.mot.set_pulse_num(pls_num=[self.pls_num], axis=self.axis)
        self.mot.set_magnification(mag=[self.mag], axis=self.axis)
        self.mot.set_fl_speed(fl_spd=[self.fl_spd], axis=self.axis)
        self.mot.set_fh_speed(fh_spd=[self.fh_spd], axis=self.axis)
        self.mot.set_acceleration_rate(acc_rate=[self.acc_rate], axis=self.axis)
        self.mot.set_deceleration_rate(dec_rate=[self.dec_rate], axis=self.axis)
        ###=== Define topic ===###
        topic_jog_onoff_cmd = '/{0}_rsw{1}_{2}_jog_onoff_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_ptp_onoff_cmd = '/{0}_rsw{1}_{2}_ptp_onoff_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_pulse_num_cmd = '/{0}_rsw{1}_{2}_pulse_num_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_pulse_num = '/{0}_rsw{1}_{2}_pulse_num'.format(self.node_name, self.rsw_id, self.axis)
        topic_fh_speed_cmd = '/{0}_rsw{1}_{2}_fh_speed_cmd'.format(self.node_name, self.rsw_id, self.axis)
        topic_fh_speed = '/{0}_rsw{1}_{2}_fh_speed'.format(self.node_name, self.rsw_id, self.axis)
        topic_onoff = '/{0}_rsw{1}_{2}_onoff'.format(self.node_name, self.rsw_id, self.axis)
        topic_move_to_home = '/{0}_rsw{1}_{2}_move_to_home'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        self.pub_pulse_num = rospy.Publisher(topic_pulse_num, Int64, queue_size=1)
        self.pub_fh_speed = rospy.Publisher(topic_fh_speed, Int64, queue_size=1)
        self.pub_onoff = rospy.Publisher(topic_onoff, Bool, queue_size=1)
        ###=== Define Subscriber ===###
        self.sub_jog_switch = rospy.Subscriber(topic_jog_onoff_cmd, Bool, self.jog_switch)
        self.sub_ptp_switch = rospy.Subscriber(topic_ptp_onoff_cmd, Bool, self.ptp_switch)
        self.sub_pulse_num_cmd = rospy.Subscriber(topic_pulse_num_cmd, Int64, self.pulse_num_cmd_switch)
        self.sub_fh_speed_cmd = rospy.Subscriber(topic_fh_speed_cmd, Int64, self.fh_speed_cmd_switch)
        self.sub_pulse_num = rospy.Subscriber(topic_pulse_num, Int64, self.pulse_num_switch)
        self.sub_move_to_home = rospy.Subscriber(topic_move_to_home, Bool, self.move_to_home_switch)

    def jog_switch(self, q):
        self.jog_flag = q.data
        return

    def ptp_switch(self, q):
        self.ptp_flag = q.data
        return

    def pulse_num_cmd_switch(self, q):
        self.pulse_num_cmd_li.append(q.data)
        self.pulse_num_cmd_flag = True
        return

    def pulse_num_switch(self, q):
        self.pulse_num_flag = q.data
        return

    def fh_speed_cmd_switch(self, q):
        self.fh_speed_cmd_li.append(q.data)
        self.fh_speed_cmd_flag = True
        return

    def fh_speed_switch(self, q):
        self.fh_speed_flag = True
        return

    def move_to_home_switch(self, q):
        self.move_to_home_flag = q.data
        return

    def move_jog(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop without pulse output ===###
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
            ###=== Stand-by loop with pulse output ===###
            while self.jog_flag == bool(self.mot.check_move_onoff(axis=self.axis)[0]):
                time.sleep(self.rate)
                continue
            ###=== End of JOG operation ===###
            self.mot.stop(axis=self.axis, check_onoff=True)
            time.sleep(self.rate)
            continue

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

    def set_pulse_num(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set pulse_num ===###
            if self.pulse_num_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set pulse_num ===###
            self.mot.set_pulse_num(axis=self.axis, pls_num=self.pulse_num_cmd_li)
            self.pulse_num_flag = True
            self.pulse_num_cmd_li = []
            self.pulse_num_cmd_flag = False
            continue

    def get_pulse_num(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get pulse_num ===###
            if self.pulse_num_flag == False:
                time.sleep(self.rate)
                continue
            ###=== publish pulse_num ===###
            pulse_num = self.mot.get_pulse_num(axis=self.axis)[0]
            self.pub_pulse_num.publish(pulse_num)
            time.sleep(self.rate)
            self.pulse_num_flag = False
            continue

    def set_fh_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for set fh_speed ===###
            if self.fh_speed_cmd_flag == False:
                time.sleep(self.rate)
                continue
            ###=== set fh_speed ===###
            self.mot.set_fh_speed(axis=self.axis, fh_spd=self.fh_speed_cmd_li)
            self.fh_speed_flag = True
            self.fh_speed_cmd_li = []
            self.fh_speed_cmd_flag = False
            continue

    def get_fh_speed(self):
        while not rospy.is_shutdown():
            ###=== Stand-by loop for get fh_speed ===###
            if self.fh_speed_flag == False:
                time.sleep(self.rate)
                continue
            ###=== publish fh_speed ===###
            fh_speed = self.mot.get_fh_speed(axis=self.axis)[0]
            self.pub_fh_speed.publish(fh_speed)
            time.sleep(self.rate)
            self.fh_speed_flag = False
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

    def check_move_onoff(self):
        self.pub_onoff.publish(self.mot.check_move_onoff(axis=self.axis)[0])
        while not rospy.is_shutdown():
            ###=== Stand-by loop without pulse output ===###
            if bool(self.mot.check_move_onoff(axis=self.axis)[0]) == False:
                time.sleep(self.rate)
                continue
            ###=== publish onoff ===###
            self.pub_onoff.publish(bool(self.mot.check_move_onoff(axis=self.axis)[0]))
            ###=== Stand-by loop with pulse output ===###
            while bool(self.mot.check_move_onoff(axis=self.axis)[0]) == True:
                time.sleep(self.rate)
                continue
            ###=== publish onoff ===###
            self.pub_onoff.publish(bool(self.mot.check_move_onoff(axis=self.axis)[0]))
            time.sleep(self.rate)
            continue

    def start_thread_ROS(self):
        th1 = threading.Thread(target=self.move_jog)
        th2 = threading.Thread(target=self.move_ptp)
        th3 = threading.Thread(target=self.set_pulse_num)
        th4 = threading.Thread(target=self.get_pulse_num)
        th5 = threading.Thread(target=self.set_fh_speed)
        th6 = threading.Thread(target=self.get_fh_speed)
        th7 = threading.Thread(target=self.check_move_onoff)
        th8 = threading.Thread(target=self.move_to_home)
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th4.setDaemon(True)
        th5.setDaemon(True)
        th6.setDaemon(True)
        th7.setDaemon(True)
        th8.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()
        th4.start()
        th5.start()
        th6.start()
        th7.start()
        th8.start()
        return

if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
