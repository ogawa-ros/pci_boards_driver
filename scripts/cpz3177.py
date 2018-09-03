#! /usr/bin/env python3

import sys
import time
import numpy
import pyinterface

import rospy
import std_msgs
from std_msgs.msg import Float64


def select_singlediff():
    parse = lambda diffch_list: diffch_list.strip('[').strip(']').split(',')
    diffch_list = list(map(int, parse(rospy.get_param('~diffch_list'))))
    ch_array = numpy.arange(1, ch_number['single'] + 1, 1).reshape(8, 8)
    get_prohibit_singlech = lambda ch: [ch_array[int((ch - 1) / 8 * 2)][ch % 8 -1],
                                        ch_array[int((ch - 1) / 8 * 2 + 1)][ch % 8 - 1]]
    remove_list = []
    [remove_list.extend(get_prohibit_singlech(ch)) for ch in diffch_list]
    singlech_list = [_ for _ in range(1, ch_number['single'] + 1)]
    [singlech_list.remove(_) for _ in remove_list]

    ret = []
    [ret.append([ch, 'single']) for ch in singlech_list]
    [ret.append([ch, 'diff']) for ch in diffch_list]

    return ret


if __name__ == '__main__':
    ch_number = {'single': 64, 'diff': 32}        
    node_name = 'cpz3177'

    rospy.init_node(node_name)
    rate = rospy.get_param('~rate')
    rsw_id = rospy.get_param('~rsw_id')        
    rate = rospy.get_param('~rate')
    rsw_id = rospy.get_param('~rsw_id')        
    
    topic_list = select_singlediff()
    pub_list = [rospy.Publisher('{0}_AIN_{1}{2}'.format(node_name, mode, ch), Float64, queue_size=1)
                       for ch, mode in topic_list]

    try:
        ad = pyinterface.open(3177, rsw_id)
    except OSError as e:
        rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".format(**locals()))
        sys.exit()

while not rospy.is_shutdown():

    for param, pub in zip(topic_list, pub_list):
        ch = param[0]
        mode = param[1]
        msg = Float64()    

        ret = ad.input_voltage(ch, mode)
            
        msg.data = ret
        pub.publish(msg)
    continue

