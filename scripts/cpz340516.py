#! /usr/bin/env python3


import sys
import time
import threading
import pyinterface

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String


ch_number = 8


def select_outputrange():
    outputrange_list = ['DA0_100mA' for _ in range(1, ch_number + 1)]
    parse = lambda da01_chlist: da01_chlist.strip('[').strip(']').split(',')
    if parse(rospy.get_param('~DA01ch_list')) == ['']: DA01ch_list = []
    else: DA01ch_list = list(map(int, parse(rospy.get_param('~DA01ch_list'))))
    for _ in DA01ch_list: outputrange_list[_-1] = 'DA0_1mA'

    return outputrange_list


class cpz340516_controller(object):

    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = rospy.get_param('~node_name')
        self.flag = 1
        self.lock = 0
        self.param_buff = []

        try:
            self.da = pyinterface.open(3405, self.rsw_id)
        except OSError as e:
            rospy.logerr('{e.strerror}. node={self.node_name}, rsw={self.rsw_id}'.format(*locals()))
            sys.exit()        

        [self.da.set_outputrange(ch, outputrange)
         for ch, outputrange in enumerate(select_outputrange(), start=1)]

        self.topic_list = [('{0}_rsw{1}_{2}_{3}'
                            .format(self.node_name, self.rsw_id, ch, outputrange))
                           for ch, outputrange in enumerate(select_outputrange(), start=1)]
        print(self.topic_list)
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
        print(req)
        print(ch)
        self.flag = 0
        pass

    def output_current(self):
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
                    self.da.output_current(ch, voltage)
                    
                elif self.lock == 1:
                    while not(self.lock == 0):
                        print('[WORNING] Lock is ON, contorol {} is prohibited.'
                              .format(self.node_name))
                        time.sleep(1)
                    self.param_buff, param_buff = [], []
                    print('[INFO] Lock is OFF, and ouput current data is deleted.')
                    break
                
                self.pub_list[ch-1].publish(voltage)
                self.flag = 1
                continue

    def start_thread_ROS(self):
        th = threading.Thread(target=self.output_current)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('cpz340516')
    ctrl = cpz340516_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
