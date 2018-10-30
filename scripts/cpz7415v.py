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
        },
        'STOP': {
            'clock': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fl_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'fh_speed': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'acc_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'dec_rate': {'x': 0, 'y': 0, 'z': 0, 'u': 0},
            'position': {'x': 0, 'y': 0, 'z': 0, 'u': 0}
        }
    }

    mode = {
        'x': 'STOP',
        'y': 'STOP',
        'z': 'STOP',
        'u': 'STOP',
    }

    do_status = {1:0, 2:0, 3:0, 4:0}

    last_position = {
        'x': 0,
        'y': 0,
        'z': 0,
        'u': 0,
    }

    last_speed = {
        'x': 0,
        'y': 0,
        'z': 0,
        'u': 0,
    }

    def __init__(self):
        ###=== Define member-variables ===###
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = rospy.get_param('~node_name')
        self.mode['x'] = rospy.get_param('~mode_x')
        self.mode['y'] = rospy.get_param('~mode_y')
        self.mode['z'] = rospy.get_param('~mode_z')
        self.mode['u'] = rospy.get_param('~mode_u')
        self.position_cmd_flag = False
        self.speed_cmd_flag = False
        self.busy_flag = False
        self.position_cmd_li = []
        self.speed_cmd_li = []
        #self.last_speed = rospy.get_param('~fh_speed')
        #self.last_position = rospy.get_param('~position')
        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        ###=== Setting the board ===###
        self.mot.initialize(axis='xyzu')
        self.motion_conf[self.mode['x']]['clock']['x'] = rospy.get_param('~clock_x')
        self.motion_conf[self.mode['y']]['clock']['y'] = rospy.get_param('~clock_y')
        self.motion_conf[self.mode['z']]['clock']['z'] = rospy.get_param('~clock_z')
        self.motion_conf[self.mode['u']]['clock']['u'] = rospy.get_param('~clock_u')
        self.motion_conf[self.mode['x']]['position']['x'] = rospy.get_param('~position_x')
        self.motion_conf[self.mode['y']]['position']['y'] = rospy.get_param('~position_y')
        self.motion_conf[self.mode['z']]['position']['z'] = rospy.get_param('~position_z')
        self.motion_conf[self.mode['u']]['position']['u'] = rospy.get_param('~position_u')
        self.motion_conf[self.mode['x']]['fl_speed']['x'] = rospy.get_param('~fl_speed_x')
        self.motion_conf[self.mode['y']]['fl_speed']['y'] = rospy.get_param('~fl_speed_y')
        self.motion_conf[self.mode['z']]['fl_speed']['z'] = rospy.get_param('~fl_speed_z')
        self.motion_conf[self.mode['u']]['fl_speed']['u'] = rospy.get_param('~fl_speed_u')
        self.motion_conf[self.mode['x']]['fh_speed']['x'] = rospy.get_param('~fh_speed_x')
        self.motion_conf[self.mode['y']]['fh_speed']['y'] = rospy.get_param('~fh_speed_y')
        self.motion_conf[self.mode['z']]['fh_speed']['z'] = rospy.get_param('~fh_speed_z')
        self.motion_conf[self.mode['u']]['fh_speed']['u'] = rospy.get_param('~fh_speed_u')
        self.motion_conf[self.mode['x']]['acc_rate']['x'] = rospy.get_param('~acc_rate_x')
        self.motion_conf[self.mode['y']]['acc_rate']['y'] = rospy.get_param('~acc_rate_y')
        self.motion_conf[self.mode['z']]['acc_rate']['z'] = rospy.get_param('~acc_rate_z')
        self.motion_conf[self.mode['u']]['acc_rate']['u'] = rospy.get_param('~acc_rate_u')
        self.motion_conf[self.mode['x']]['dec_rate']['x'] = rospy.get_param('~dec_rate_x')
        self.motion_conf[self.mode['y']]['dec_rate']['y'] = rospy.get_param('~dec_rate_y')
        self.motion_conf[self.mode['z']]['dec_rate']['z'] = rospy.get_param('~dec_rate_z')
        self.motion_conf[self.mode['u']]['dec_rate']['u'] = rospy.get_param('~dec_rate_u')
        self.mot.set_motion(axis='x', mode=self.mode['x'])
        self.mot.set_motion(axis='y', mode=self.mode['y'])
        self.mot.set_motion(axis='z', mode=self.mode['z'])
        self.mot.set_motion(axis='u', mode=self.mode['u'])
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
        topic_output_do1_cmd = '/{0}_rsw{1}_do1_cmd'.format(self.node_name, self.rsw_id)
        topic_output_do2_cmd = '/{0}_rsw{1}_do2_cmd'.format(self.node_name, self.rsw_id)
        topic_output_do3_cmd = '/{0}_rsw{1}_do3_cmd'.format(self.node_name, self.rsw_id)
        topic_output_do4_cmd = '/{0}_rsw{1}_do4_cmd'.format(self.node_name, self.rsw_id)
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
        self.sub_position_x_cmd = rospy.Subscriber(topic_position_x_cmd, Int64, self.set_position, callback_args='x')
        self.sub_position_y_cmd = rospy.Subscriber(topic_position_y_cmd, Int64, self.set_position, callback_args='y')
        self.sub_position_z_cmd = rospy.Subscriber(topic_position_z_cmd, Int64, self.set_position, callback_args='z')
        self.sub_position_u_cmd = rospy.Subscriber(topic_position_u_cmd, Int64, self.set_position, callback_args='u')
        self.sub_speed_x_cmd = rospy.Subscriber(topic_speed_x_cmd, Int64, self.set_speed, callback_args='x')
        self.sub_speed_y_cmd = rospy.Subscriber(topic_speed_y_cmd, Int64, self.set_speed, callback_args='y')
        self.sub_speed_z_cmd = rospy.Subscriber(topic_speed_z_cmd, Int64, self.set_speed, callback_args='z')
        self.sub_speed_u_cmd = rospy.Subscriber(topic_speed_u_cmd, Int64, self.set_speed, callback_args='u')
        self.sub_output_do1_cmd = rospy.Subscriber(topic_output_do1_cmd, Bool, self.output_do, callback_args=1)
        self.sub_output_do2_cmd = rospy.Subscriber(topic_output_do2_cmd, Bool, self.output_do, callback_args=2)
        self.sub_output_do3_cmd = rospy.Subscriber(topic_output_do3_cmd, Bool, self.output_do, callback_args=3)
        self.sub_output_do4_cmd = rospy.Subscriber(topic_output_do4_cmd, Bool, self.output_do, callback_args=4)


    def set_position(self, q, axis):
        self.motion_conf['PTP']['position'][axis] = q.data
        return


    def _set_position(self):
        self.mot.driver.pMotion = self.motion_conf
        
        if (self.mode['x'] == 'PTP' and 
            self.mot.driver.check_move_onoff('x')[0] == 0):
            self.mot.set_motion(axis='x', mode='PTP')
            self.mot.start_motion(axis='x', stamod='staud', movmod='PTP')
        else:
            pass
        
        if (self.mode['y'] == 'PTP' and 
            self.mot.driver.check_move_onoff('y')[0] == 0):
            self.mot.set_motion(axis='y', mode='PTP')
            self.mot.start_motion(axis='y', stamod='staud', movmod='PTP')
        else:
            pass
        
        if (self.mode['z'] == 'PTP' and 
            self.mot.driver.check_move_onoff('z')[0] == 0):
            self.mot.set_motion(axis='z', mode='PTP')
            self.mot.start_motion(axis='z', stamod='staud', movmod='PTP')
        else:
            pass
        
        if (self.mode['u'] == 'PTP' and 
            self.mot.driver.check_move_onoff('u')[0] == 0):
            self.mot.set_motion(axis='u', mode='PTP')
            self.mot.start_motion(axis='u', stamod='staud', movmod='PTP')
        else:
            pass
        
        return


    def _get_position(self):
        position_x = self.mot.read_counter(axis='x')[0]
        position_y = self.mot.read_counter(axis='y')[0]
        position_z = self.mot.read_counter(axis='z')[0]
        position_u = self.mot.read_counter(axis='u')[0]
        
        if self.last_position['x'] != position_x: self.pub_position_x.publish(position_x)
        if self.last_position['y'] != position_y: self.pub_position_y.publish(position_y)
        if self.last_position['z'] != position_z: self.pub_position_z.publish(position_z)
        if self.last_position['u'] != position_u: self.pub_position_u.publish(position_u)
        
        self.last_position['x'] = position_x
        self.last_position['y'] = position_y
        self.last_position['z'] = position_z
        self.last_position['u'] = position_u
        return


    def set_speed(self, q, axis):
        self.motion_conf['JOG']['fh_speed'][axis] = q.data
        return


    def _set_speed(self):
        self.mot.driver.pMotion = self.motion_conf

        if self.mode['x'] == 'JOG': self.mot.change_speed('x', [self.motion_conf['JOG']['speed']['x']])
        if self.mode['y'] == 'JOG': self.mot.change_speed('y', [self.motion_conf['JOG']['speed']['y']])
        if self.mode['z'] == 'JOG': self.mot.change_speed('z', [self.motion_conf['JOG']['speed']['z']])
        if self.mode['u'] == 'JOG': self.mot.change_speed('u', [self.motion_conf['JOG']['speed']['u']])
        return


    def _get_speed(self):
        speed_x = self.mot.read_speed('x')[0]
        speed_y = self.mot.read_speed('y')[0]
        speed_z = self.mot.read_speed('z')[0]
        speed_u = self.mot.read_speed('u')[0]

        if self.last_speed['x'] != speed_x: self.pub_speed_x.publish(speed_x)
        if self.last_speed['y'] != speed_y: self.pub_speed_y.publish(speed_y)
        if self.last_speed['z'] != speed_z: self.pub_speed_z.publish(speed_z)
        if self.last_speed['u'] != speed_u: self.pub_speed_u.publish(speed_u)

        self.last_speed['x'] = speed_x
        self.last_speed['y'] = speed_y
        self.last_speed['z'] = speed_z
        self.last_speed['u'] = speed_u
        return


    def output_do(self, q, ch):
        self.do_status[ch] = q.data
        return
    
    
    def _output_do(self):
        self.mot.driver._output_do(1, int(self.do_status[1]))
        self.mot.driver._output_do(2, int(self.do_status[2]))
        self.mot.driver._output_do(3, int(self.do_status[3]))
        self.mot.driver._output_do(4, int(self.do_status[4]))
        return
    

    def _main_thread(self):
        while not rospy.is_shutdown():
            self._set_position()
            self._set_speed()
            self._get_position()
            self._get_speed()
            self._output_do()
            continue


    def start_thread_ROS(self):
        th = threading.Thread(target=self._main_thread)
        th.setDaemon(True)
        th.start()
        return


if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    #ctrl.mot.start_motion(axis=ctrl.axis, stamod='staud', movmod=ctrl.mode)
    rospy.spin()
