#! /usr/bin/env python3

name = "cpz2724m_antenna"

#----
import sys
import time
import threading
import rospy
import std_msgs.msg
import pyinterface


class CPZ2724(object):

    def __init__(self):
        self.rsw_id = rospy.get_param("~rsw_id")

        self.sub_az = rospy.Subscriber(
                        name = "cpz2724_rsw{}/do01_16".format(self.rsw_id),
                        data_class = std_msgs.msg.ByteMultiArray,
                        callback = self.output_function_az,
                        queue_size = 1 
                    )

        self.sub_el = rospy.Subscriber(
                        name = "cpz2724_rsw{}/do17_32".format(self.rsw_id),
                        data_class = std_msgs.msg.ByteMultiArray,
                        callback = self.output_function_el,
                        queue_size = 1 
                    )

        try:
            self.dio = pyinterface.open(2724, self.rsw_id)
        except OSError as e:
            rospy.logerr(e, name, self.rsw_id)
            sys.exit()
        
        pass

    def output_function_az(self, req):
        self.dio.output_word('OUT1_16', req.data)
        return

    def output_function_el(self, req):
        self.dio.output_word('OUT17_32', req.data)
        return

if __name__ == "__main__":
    rospy.init_node(name)
    cpz = CPZ2724()

    rospy.spin()
