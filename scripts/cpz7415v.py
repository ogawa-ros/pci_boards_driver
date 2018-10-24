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

    motion_conf = {
        'JOG': {
            'clock': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fl_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fh_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'acc_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'dec_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'position': {'x': 0, 'y': 0, 'z': 0, 'u': 0}
        },
        'PTP': {
            'clock': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fl_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fh_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'acc_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'dec_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'position': {'x': 0, 'y': 0, 'z': 0, 'u': 0}
        }
    }

    def __init__(self):
        ###=== Define member-variables ===###
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = rospy.get_param('~node_name')
        self.mode_x = rospy.get_param('~mode_x')
        self.mode_y = rospy.get_param('~mode_y')
        self.mode_z = rospy.get_param('~mode_z')
        self.mode_u = rospy.get_param('~mode_u')
        self.position_cmd_flag = False
        self.speed_cmd_flag = False
        self.busy_flag = False
        self.position_cmd_li = []
        self.speed_cmd_li = []
        self.last_speed = rospy.get_param('~fh_speed')
        self.last_position = rospy.get_param('~position')
        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        ###=== Setting the board ===###
        self.mot.initialize(axis='xyzu')
        self.motion_conf[self.mode]['clock']['x'] = rospy.get_param('~clock_x')
        self.motion_conf[self.mode]['clock']['y'] = rospy.get_param('~clock_y')
        self.motion_conf[self.mode]['clock']['z'] = rospy.get_param('~clock_z')
        self.motion_conf[self.mode]['clock']['u'] = rospy.get_param('~clock_u')
        self.motion_conf[self.mode]['position']['x'] = rospy.get_param('~position_x')
        self.motion_conf[self.mode]['position']['y'] = rospy.get_param('~position_y')
        self.motion_conf[self.mode]['position']['z'] = rospy.get_param('~position_z')
        self.motion_conf[self.mode]['position']['u'] = rospy.get_param('~position_u')
        self.motion_conf[self.mode]['fl_speed']['x'] = rospy.get_param('~fl_speed_x')
        self.motion_conf[self.mode]['fl_speed']['y'] = rospy.get_param('~fl_speed_y')
        self.motion_conf[self.mode]['fl_speed']['z'] = rospy.get_param('~fl_speed_z')
        self.motion_conf[self.mode]['fl_speed']['u'] = rospy.get_param('~fl_speed_u')
        self.motion_conf[self.mode]['fh_speed']['x'] = rospy.get_param('~fh_speed_x')
        self.motion_conf[self.mode]['fh_speed']['y'] = rospy.get_param('~fh_speed_y')
        self.motion_conf[self.mode]['fh_speed']['z'] = rospy.get_param('~fh_speed_z')
        self.motion_conf[self.mode]['fh_speed']['u'] = rospy.get_param('~fh_speed_u')
        self.motion_conf[self.mode]['acc_rate']['x'] = rospy.get_param('~acc_rate_x')
        self.motion_conf[self.mode]['acc_rate']['y'] = rospy.get_param('~acc_rate_y')
        self.motion_conf[self.mode]['acc_rate']['z'] = rospy.get_param('~acc_rate_z')
        self.motion_conf[self.mode]['acc_rate']['u'] = rospy.get_param('~acc_rate_u')
        self.motion_conf[self.mode]['dec_rate']['x'] = rospy.get_param('~dec_rate_x')
        self.motion_conf[self.mode]['dec_rate']['y'] = rospy.get_param('~dec_rate_y')
        self.motion_conf[self.mode]['dec_rate']['z'] = rospy.get_param('~dec_rate_z')
        self.motion_conf[self.mode]['dec_rate']['u'] = rospy.get_param('~dec_rate_u')
        self.mot.set_motion(axis=self.axis, mode=self.mode)
        ###=== Define topic ===###
        topic_position_x_cmd = '/{0}_rsw{1}_x_position_cmd'.format(self.node_name, self.rsw_id)
        topic_position_y_cmd = '/{0}_rsw{1}_y_position_cmd'.format(self.node_name, self.rsw_id)
        topic_position_z_cmd = '/{0}_rsw{1}_z_position_cmd'.format(self.node_name, self.rsw_id)
        topic_position_u_cmd = '/{0}_rsw{1}_u_position_cmd'.format(self.node_name, self.rsw_id)
        topic_position_x = '/{0}_rsw{1}_x_position'.format(self.node_name, self.rsw_id)
        topic_position_y = '/{0}_rsw{1}_y_position'.format(self.node_name, self.rsw_id)
        topic_position_z = '/{0}_rsw{1}_z_position'.format(self.node_name, self.rsw_id)
        topic_position_u = '/{0}_rsw{1}_u_position'.format(self.node_name, self.rsw_id)
        topic_speed_x_cmd = '/{0}_rsw{1}_x_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_y_cmd = '/{0}_rsw{1}_y_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_z_cmd = '/{0}_rsw{1}_z_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_u_cmd = '/{0}_rsw{1}_u_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_x = '/{0}_rsw{1}_x_speed'.format(self.node_name, self.rsw_id)
        topic_speed_y = '/{0}_rsw{1}_y_speed'.format(self.node_name, self.rsw_id)
        topic_speed_z = '/{0}_rsw{1}_z_speed'.format(self.node_name, self.rsw_id)
        topic_speed_u = '/{0}_rsw{1}_u_speed'.format(self.node_name, self.rsw_id)
        topic_output_do_cmd = '/{0}_rsw{1}_{2}_output_do_cmd'.format(self.node_name, self.rsw_id, self.axis)
        ###=== Define Publisher ===###
        self.pub_position_x = rospy.Publisher(topic_position_x, Int64, queue_size=1)
        self.pub_position_y = rospy.Publisher(topic_position_y, Int64, queue_size=1)
        self.pub_position_z = rospy.Publisher(topic_position_z, Int64, queue_size=1)
        self.pub_position_u = rospy.Publisher(topic_position_u, Int64, queue_size=1)
        self.pub_speed_x = rospy.Publisher(topic_speed_x, Int64, queue_size=1)
        self.pub_speed_y = rospy.Publisher(topic_speed_y, Int64, queue_size=1)
        self.pub_speed_z = rospy.Publisher(topic_speed_z, Int64, queue_size=1)
        self.pub_speed_u = rospy.Publisher(topic_speed_u, Int64, queue_size=1)
        ###=== Define Subscriber ===###
        self.sub_position_x_cmd = rospy.Subscriber(topic_position_x_cmd, Int64, self.position_cmd_switch, callback_args='x')
        self.sub_position_y_cmd = rospy.Subscriber(topic_position_y_cmd, Int64, self.position_cmd_switch, callback_args='y')
        self.sub_position_z_cmd = rospy.Subscriber(topic_position_z_cmd, Int64, self.position_cmd_switch, callback_args='z')
        self.sub_position_u_cmd = rospy.Subscriber(topic_position_u_cmd, Int64, self.position_cmd_switch, callback_args='u')
        self.sub_speed_x_cmd = rospy.Subscriber(topic_speed_x_cmd, Int64, self.speed_cmd_switch, callback_args='x')
        self.sub_speed_y_cmd = rospy.Subscriber(topic_speed_y_cmd, Int64, self.speed_cmd_switch, callback_args='y')
        self.sub_speed_z_cmd = rospy.Subscriber(topic_speed_z_cmd, Int64, self.speed_cmd_switch, callback_args='z')
        self.sub_speed_u_cmd = rospy.Subscriber(topic_speed_u_cmd, Int64, self.speed_cmd_switch, callback_args='u')
        self.sub_output_do_cmd = rospy.Subscriber(topic_output_do_cmd, Int64, self.output_do_cmd_switch)


    def set_position(self, q, axis):
        self.motion_conf[self.mode]['position'][axis] = q.data
        return


    def _set_position(self):
        self.mot.driver.pMotion = self.motion_conf
        if self.mot.driver.check_move_onoff('x')[0] == 0:
            self.mot.set_motion(axis=self.axis, mode='PTP')
            self.mot.start_motion(axis=self.axis, stamod='staud', movmod='PTP')
        else: pass
        return


    def get_position(self):
        position = self.mot.read_counter(axis=self.axis)[0]
        self.pub_position.publish(position)
        last_position = position
        time.sleep(self.rate)
        return


    def set_speed(self, q, axis):
        self.mot.change_speed(axis, spd=q.data)
        return


    def _set_speed(self):
        self.mot.driver.pMotion = self.motion_conf
        self.mot.change_speed(axis, [self.motion_conf['JOG']['speed'][axis]])
        return


    def get_speed(self):
        speed = self.mot.read_speed(axis=self.axis)[0]
        self.pub_speed.publish(speed)
        time.sleep(self.rate)
        return


    def param_io(self):
        while not rospy.is_shutdown():
            if self.mode == 'PTP':
                self._set_position()
            if self.mode == 'JOG':
                self._set_speed()
            #get


    def start_thread_ROS(self):
        th = threading.Thread(target=self.set_position)
        th.setDaemon(True)
        th.start()
        return


if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    ctrl.mot.start_motion(axis=ctrl.axis, stamod='staud', movmod=ctrl.mode)
    rospy.spin()
