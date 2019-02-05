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

    motion = {
        'x': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'y': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'z': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'u': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        }
    }

    move_mode = {'x': '', 'y': '', 'z': '', 'u': ''}

    last_position = {'x': 0, 'y': 0, 'z': 0, 'u': 0}

    last_speed = {'x': 0, 'y': 0, 'z': 0, 'u': 0}

    do_status = 0

    step_flag = {'x': False, 'y': False, 'z': False, 'u': False}
    speed_flag = {'x': False, 'y': False, 'z': False, 'u': False}
    do_flag = False

    
    def __init__(self):
        ###=== Define member-variables ===###
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = rospy.get_param('~node_name')
        self.move_mode['x'] = rospy.get_param('~mode_x')
        self.move_mode['y'] = rospy.get_param('~mode_y')
        self.move_mode['z'] = rospy.get_param('~mode_z')
        self.move_mode['u'] = rospy.get_param('~mode_u')

        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()

        ###=== Setting the board ===###
        self.mot.initialize(axis='xyzu')
        self.motion['x']['clock'] = rospy.get_param('~clock_x')
        self.motion['x']['acc_mode'] = rospy.get_param('~acc_mode_x')
        self.motion['x']['low_speed'] = rospy.get_param('~low_speed_x')
        self.motion['x']['speed'] = rospy.get_param('~speed_x')
        self.motion['x']['acc'] = rospy.get_param('~acc_x')
        self.motion['x']['dec'] = rospy.get_param('~dec_x')
        self.motion['x']['step'] = rospy.get_param('~step_x')
        self.motion['y']['clock'] = rospy.get_param('~clock_y')
        self.motion['y']['acc_mode'] = rospy.get_param('~acc_mode_y')
        self.motion['y']['low_speed'] = rospy.get_param('~low_speed_y')
        self.motion['y']['speed'] = rospy.get_param('~speed_y')
        self.motion['y']['acc'] = rospy.get_param('~acc_y')
        self.motion['y']['dec'] = rospy.get_param('~dec_y')
        self.motion['y']['step'] = rospy.get_param('~step_y')
        self.motion['z']['clock'] = rospy.get_param('~clock_z')
        self.motion['z']['acc_mode'] = rospy.get_param('~acc_mode_z')
        self.motion['z']['low_speed'] = rospy.get_param('~low_speed_z')
        self.motion['z']['speed'] = rospy.get_param('~speed_z')
        self.motion['z']['acc'] = rospy.get_param('~acc_z')
        self.motion['z']['dec'] = rospy.get_param('~dec_z')
        self.motion['z']['step'] = rospy.get_param('~step_z')
        self.motion['u']['clock'] = rospy.get_param('~clock_u')
        self.motion['u']['acc_mode'] = rospy.get_param('~acc_mode_u')
        self.motion['u']['low_speed'] = rospy.get_param('~low_speed_u')
        self.motion['u']['speed'] = rospy.get_param('~speed_u')
        self.motion['u']['acc'] = rospy.get_param('~acc_u')
        self.motion['u']['dec'] = rospy.get_param('~dec_u')
        self.motion['u']['step'] = rospy.get_param('~step_u')
        self.mot.set_motion(axis='x', mode=self.move_mode['x'], motion=self.motion)
        self.mot.set_motion(axis='y', mode=self.move_mode['y'], motion=self.motion)
        self.mot.set_motion(axis='z', mode=self.move_mode['z'], motion=self.motion)
        self.mot.set_motion(axis='u', mode=self.move_mode['u'], motion=self.motion)

        ###=== Define topic ===###
        topic_step_x_cmd = '/{0}_rsw{1}_x_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_x = '/{0}_rsw{1}_x_step'.format(self.node_name, self.rsw_id)
        topic_speed_x_cmd = '/{0}_rsw{1}_x_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_x = '/{0}_rsw{1}_x_speed'.format(self.node_name, self.rsw_id)
        topic_step_y_cmd = '/{0}_rsw{1}_y_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_y = '/{0}_rsw{1}_y_step'.format(self.node_name, self.rsw_id)
        topic_speed_y_cmd = '/{0}_rsw{1}_y_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_y = '/{0}_rsw{1}_y_speed'.format(self.node_name, self.rsw_id)
        topic_step_z_cmd = '/{0}_rsw{1}_z_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_z = '/{0}_rsw{1}_z_step'.format(self.node_name, self.rsw_id)
        topic_speed_z_cmd = '/{0}_rsw{1}_z_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_z = '/{0}_rsw{1}_z_speed'.format(self.node_name, self.rsw_id)
        topic_step_u_cmd = '/{0}_rsw{1}_u_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_u = '/{0}_rsw{1}_u_step'.format(self.node_name, self.rsw_id)
        topic_speed_u_cmd = '/{0}_rsw{1}_u_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_u = '/{0}_rsw{1}_u_speed'.format(self.node_name, self.rsw_id)
        topic_output_do_cmd = '/{0}_rsw{1}_do_cmd'.format(self.node_name, self.rsw_id)

        ###=== Define Publisher ===###
        self.pub_step_x = rospy.Publisher(topic_step_x, Int64, queue_size=1)
        self.pub_speed_x = rospy.Publisher(topic_speed_x, Int64, queue_size=1)
        self.pub_step_y = rospy.Publisher(topic_step_y, Int64, queue_size=1)
        self.pub_speed_y = rospy.Publisher(topic_speed_y, Int64, queue_size=1)
        self.pub_step_z = rospy.Publisher(topic_step_z, Int64, queue_size=1)
        self.pub_speed_z = rospy.Publisher(topic_speed_z, Int64, queue_size=1)
        self.pub_step_u = rospy.Publisher(topic_step_u, Int64, queue_size=1)
        self.pub_speed_u = rospy.Publisher(topic_speed_u, Int64, queue_size=1)

        ###=== Define Subscriber ===###
        self.sub_step_x_cmd = rospy.Subscriber(topic_step_x_cmd, Int64, self.set_step, callback_args='x')
        self.sub_speed_x_cmd = rospy.Subscriber(topic_speed_x_cmd, Int64, self.set_speed, callback_args='x')
        self.sub_step_y_cmd = rospy.Subscriber(topic_step_y_cmd, Int64, self.set_step, callback_args='y')
        self.sub_speed_y_cmd = rospy.Subscriber(topic_speed_y_cmd, Int64, self.set_speed, callback_args='y')
        self.sub_step_z_cmd = rospy.Subscriber(topic_step_z_cmd, Int64, self.set_step, callback_args='z')
        self.sub_speed_z_cmd = rospy.Subscriber(topic_speed_z_cmd, Int64, self.set_speed, callback_args='z')
        self.sub_step_u_cmd = rospy.Subscriber(topic_step_u_cmd, Int64, self.set_step, callback_args='u')
        self.sub_speed_u_cmd = rospy.Subscriber(topic_speed_u_cmd, Int64, self.set_speed, callback_args='u')
        self.sub_output_do_cmd = rospy.Subscriber(topic_output_do_cmd, Int64, self.output_do)

        pass


    def set_step(self, q, axis):
        self.motion[axis]['step'] = q.data
        self.step_flag[axis] = True
        return

    def _set_step(self):
        axis = ''
        step = []
        for i in self.move_mode:
            if self.step_flag[i]:
                axis += i
                step.append(self.motion[i]['step'])
            else: pass

        if axis != '':
            self.mot.set_motion(axis=axis, mode='ptp', motion=self.motion)
            self.mot.start_motion(axis=axis, start_mode='acc', move_mode='ptp')
            for i in axis:
                self.step_flag[i] = False
            time.sleep(0.0001)
        else: pass

        return

    def _get_step(self):
        step = self.mot.read_counter(axis='xyzu', cnt_mode='counter')

        if self.last_position['x'] != step[0]:
            self.pub_step_x.publish(step[0])
            self.last_position['x'] = step[0]
        if self.last_position['y'] != step[1]:
            self.pub_step_y.publish(step[1])
            self.last_position['y'] = step[1]
        if self.last_position['z'] != step[2]:
            self.pub_step_z.publish(step[2])
            self.last_position['z'] = step[2]
        if self.last_position['u'] != step[3]:
            self.pub_step_u.publish(step[3])
            self.last_position['u'] = step[3]

        time.sleep(0.0001)
        return


    def set_speed(self, q, axis):
        self.motion[axis]['speed'] = q.data
        self.speed_flag[axis] = True
        return


    def _set_speed(self):
        axis = ''
        speed = []
        for i in self.move_mode:
            if self.speed_flag[i]:
                axis += i
                speed.append(self.motion[i]['speed'])
            else: pass

        if axis != '':
            self.mot.set_motion(axis=axis, mode='jog', motion=self.motion)
            self.mot.change_speed(axis=axis, mode='accdec_change', speed=speed)
            for i in axis:
                self.speed_flag[i] = False
            time.sleep(0.0001)
        else: pass

        return


    def _get_speed(self):
        speed = self.mot.read_speed(axis='xyzu')

        if self.last_speed['x'] != speed[0]:
            self.pub_speed_x.publish(speed[0])
            self.last_speed['x'] = speed[0]
        if self.last_speed['y'] != speed[1]:
            self.pub_speed_y.publish(speed[1])
            self.last_speed['y'] = speed[1]
        if self.last_speed['z'] != speed[2]:
            self.pub_speed_z.publish(speed[2])
            self.last_speed['z'] = speed[2]
        if self.last_speed['u'] != speed[3]:
            self.pub_speed_u.publish(speed[3])
            self.last_speed['u'] = speed[3]

        time.sleep(0.0001)
        return


    def output_do(self, q):
        self.do_status = q.data
        self.do_flag = True
        return
    
    
    def _output_do(self):
        self.mot.output_do(self.do_status)
        self.do_flag = False
        time.sleep(0.001)
        return
    

    def _main_thread(self):
        while not rospy.is_shutdown():
            self._set_step()
            self._set_speed()
            self._get_step()
            self._get_speed()
            if self.do_flag:
                self._output_do()
            continue
        return


    def start_thread_ROS(self):
        th = threading.Thread(target=self._main_thread)
        th.setDaemon(True)
        th.start()
        return


if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
