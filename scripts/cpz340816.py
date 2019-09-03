#! /usr/bin/env python3


import sys
import time
import threading
import pyinterface

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64


ch_number = 16


class cpz340816_controller(object):

    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = 'cpz340816'
        self.param_buff = []

        try:
            self.da = pyinterface.open(3408, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()

        self.topic_list = [('{0}_rsw{1}_{2}'.format(self.node_name, self.rsw_id, _))
                           for _ in range(1, ch_number + 1)]

        self.pub_list = [rospy.Publisher(
            name = topic,
            data_class = Float64,
            latch = True,
            # queue_size = 1
            ) for topic in self.topic_list]

        self.sub_list = [rospy.Subscriber(
            name = topic + '_cmd',
            data_class = Float64,
            callback = self.set_param,
            callback_args = ch,
            # queue_size = 1
            ) for ch, topic in enumerate(self.topic_list, start=1)]


    def set_param(self, req, ch):
        self.param_buff.append({'{}'.format(ch): req.data})
        pass

    def output_voltage(self):
        while not rospy.is_shutdown():

            if len(self.param_buff) < 0:
                time.sleep(self.rate)
                continue

            param_buff = self.param_buff.copy()
            self.param_buff = []

            for param in param_buff:
                ch = int(list(param.keys())[0])
                voltage = list(param.values())[0]

                self.da.output_voltage(ch, voltage)
                self.pub_list[ch-1].publish(voltage)


    def start_thread_ROS(self):
        th = threading.Thread(target=self.output_voltage)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('cpz340816')
    ctrl = cpz340816_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
