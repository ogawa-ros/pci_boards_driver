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
        self.node_name = rospy.get_param('~node_name')
        self.flag = 1
        self.lock = 0
        self.param_buff = []        

        try:
            self.da = pyinterface.open(3408, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
        
        self.topic_list = [('{0}_rsw{1}_{2}'.format(self.node_name, self.rsw_id, _))
                           for _ in range(1, ch_number + 1)]
        self.pub_list = [rospy.Publisher(topic, Float64, queue_size=1)
                         for topic in self.topic_list]
        self.sub_list = [rospy.Subscriber(topic+'_cmd', Float64, self.set_param, callback_args=ch)
                         for ch, topic in enumerate(self.topic_list, start=1)]
        self.sub_lock = rospy.Subscriber('{0}_rsw{1}_lock'
                                         .format(self.node_name, self.rsw_id), Int32, self.set_lock)
        
    def set_lock(self, req):
        self.lock = req.data
        pass
        
    def set_param(self, req, ch):
        self.param_buff.append({'{}'.format(ch): req.data})
        self.flag = 0
        pass
            
    def output_voltage(self):
        while not rospy.is_shutdown():

            if self.flag == 1:
                time.sleep(self.rate)
                continue
            
            param_buff = self.param_buff.copy()
            self.param_buff = []
            
            for param in param_buff:
                ch = int(list(param.keys())[0])
                voltage = list(param.values())[0]
                
                if self.lock == 0:
                    self.da.output_voltage(ch, voltage)
                    
                elif self.lock == 1:
                    while not(self.lock == 0):
                        print('[WORNING] Lock is ON, contorol {} is prohibited.'
                              .format(self.node_name))
                        time.sleep(1)
                    self.param_buff, param_buff = [], []
                    print('[INFO] Lock is OFF, and ouput voltage data is deleted.')
                    break
                
                self.pub_list[ch-1].publish(voltage)
                self.flag = 1
                continue

    def start_thread_ROS(self):
        th = threading.Thread(target=self.output_voltage)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('cpz340816')
    ctrl = cpz340816_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
