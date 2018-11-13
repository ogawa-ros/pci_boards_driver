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

    origin = None
    origin_flag = False

    counter = {"counter": None, "ch": None}
    counter_flag = False

    def __init__(self):
        self.ch_list = rospy.get_param("~ch_list")
        self.ch_list_set = rospy.get_param("~ch_list_set")
        self.rsw_id = rospy.get_param("~rsw_id")

        self.pub = [rospy.Publisher(
                        name = "/cpz6204_rsw{0}/ch{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Int64,
                        latch = True,
                        queue_size = 1,
                    ) for ch in self.ch_list]
        
        self.sub = [rospy.Subscriber(
                        name = "/cpz6204_rsw{0}/ch{1}_set_counter".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Int64,
                        callback = self.set_counter,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list_set]

        if self.rsw_id == "0":
            self.sub_origin = rospy.Subscriber(
                    name = "/cpz6204_rsw{0}/origin",
                    data_class = std_msgs.msg.Bool,
                    callback = self.set_origin,
                    queue_size = 1,
                )
        
        try:
            self.dio = pyinterface.open(6204, self.rsw_id)
            self.initialize()
        except OSError as e:
            rospy.logerr(e, name, self.rsw_id)
            sys.exit()
        
        pass


    def initialize(self):
        if self.rsw_id == "0":
            self.dio.initialize()
            self.board_setting()
        elif self.rsw_id == "1":
            self.dio.reset(ch=1)
            self.dio.set_mode(mode="MD0", 0, 1, 0, ch=1)
        else:pass

        return

    def board_setting(self, z_mode=""):
        [self.dio.set_mode(mode="MD0 SEL1", direction=1, equal=0, latch=0, ch=ch) 
                for ch in self.ch_list]
        [self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=ch) 
                for ch in self.ch_list]
        return

    # for encoder

    def set_origin(self, req):
        self.origin = req.data
        self.origin_flag = True
        return

    def origin_setting(self):
        while not rospy.is_shutdown():
            if not self.origin_flag:
                time.sleep(0.01)
                continue

            if self.origin == True:
                self.board_setting(z_mode="CLS0")
            else:
                self.board_setting(z_mode="")
            self.origin_flag = False
            time.sleep(0.1)
        return

    # for dome_encoder
    def set_counter(self, counter, ch):
        self.counter["counter"] = counter.data
        self.counter["ch"] = ch
        self.counter_flag = True
        return

    def counter_setting(self):
        while not rospy.is_shutdown():
            if not self.counter_flag:
                time.sleep(0.01)
                continue
            
            self.dio.set_counter(self.counter["counter"], ch=self.counter["ch"])
            self.counter_flag = False
            time.sleep(0.1)
        return

    def pub_function(self):
        while not rospy.is_shutdown():
            for ch, pub in zip(self.ch_list, self.pub):
                ret = self.dio.get_counter(ch).to_int()
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
    origin_thread = threading.Thread(
            target = cpz.origin_setting,
            daemon = True,
        )
    counter_thread = threading.Thread(
            target = cpz.counter_setting,
            daemon = True,
        )
    pub_thread.start()
    origin_thread.start()
    counter_thread.start()

    rospy.spin()
