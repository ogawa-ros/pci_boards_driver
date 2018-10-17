#! /usr/bin/env python3

name = "cpz6204"

#----
import sys
import time
import threading
import rospy
import std_msgs.msg
import pyinterface


class CPZ6204(object):

    def __init__(self):
        self.ch_list = rospy.get_param("~ch_list")
        self.rsw_id = rospy.get_param("~rsw_id")

        self.pub = [rospy.Publisher(
                        name = "/cpz6204_rsw{0}/ch{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        latch = True,
                        queue_size = 1,
                    ) for ch in self.ch_list]
        
        try:
            self.dio = pyinterface.open(6204, self.rsw_id)
        except OSError as e:
            rospy.logerr(e, name, self.rsw_id)
            sys.exit()
        
        pass

    def pub_function(self):
        while not rospy.is_shutdown():
            for ch, pub in zip(self.ch_list, self.pub):
                ret = self.dio.get_counter(ch)
                pub.publish(int(ret))
            
            time.sleep(0.001)
            continue
        
        return


if __name__ == "__main__":
    rospy.init_node(name)
    cpz = CPZ6204()

    pub_thread = threading.Thread(
            target = cpz.pub_function,
            daemon = True,
        )
    pub_thread.start()

    rospy.spin()
