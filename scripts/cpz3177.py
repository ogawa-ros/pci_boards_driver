#! /usr/bin/env python3

import sys
import time
import pyinterface

import rospy
import std_msgs
from std_msgs.msg import Float64


def str2list(param):
    _list = [_.strip(' ') for _ in list(param.strip('[').strip(']').split(','))]
    ret = _list
    ret[1] = _list[1][1:-1]
    ret[2] = int(_list[2])

    return list(ret)


if __name__ == '__main__':
    ch_number = 64
    node_name = 'cpz3177'    
    topic_name_index = 0
    inputmode_index = 1
    onoff_index = 2

    rospy.init_node(node_name)

    rsw_id = rospy.get_param('~rsw_id')
    topic_list = [str2list(rospy.get_param('~topic{}'.format(i+1)))
                       for i in range(ch_number)]
    pub_list = [rospy.Publisher(topic[topic_name_index], Float64, queue_size=1) \
                     for topic in topic_list if int(topic[onoff_index]) == 1]
    param_list = [[topic[topic_name_index], topic[inputmode_index]]
                       for topic in topic_list if int(topic[onoff_index]) ==1]
    msg_list = [Float64() for i in range(len(param_list))]

    try:
        ad = pyinterface.open(3177, rsw_id)
    except OSError as e:
        rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".format(**locals()))
        sys.exit()

while not rospy.is_shutdown():

        for pub, msg, param in zip(pub_list, msg_list, param_list):
            topic = param[0]
            inputrange = param[1]
            ch = int(topic.replace('cpz3177_ch', ''))
            
            ret = ad.input_voltage(ch, inputrange)
            
            msg.data = ret
            pub.publish(msg)
        continue
