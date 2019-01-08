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
        self.rsw_id = rospy.get_param("~rsw_id")
        if self.rsw_id == 0:
            self.ch_list = [1,2]
            self.ch_list_set = ""
        if self.rsw_id == 1:
            self.ch_list = [1]
            self.ch_list_set = [1]

        self.pub = [rospy.Publisher(
                        name = "/cpz6204_rsw{0}/ch0{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Int64,
                        latch = True,
                        queue_size = 1,
                    ) for ch in self.ch_list]
        
        self.sub = [rospy.Subscriber(
                        name = "/cpz6204_rsw{0}/ch0{1}_set_counter".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Int64,
                        callback = self.set_counter,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list_set]

        if self.rsw_id == 0:
            self.sub_origin = rospy.Subscriber(
                    name = "/cpz6204_rsw{0}/origin".format(self.rsw_id),
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
        if self.rsw_id == 0:
            mode = self.dio.get_mode()
            if mode["mode"] == "":
                self.dio.initialize()
                [self.dio.set_mode(mode="MD0 SEL1", direction=1, equal=0, latch=0, ch=ch) 
                        for ch in self.ch_list]
                self.board_setting()
            else: pass
        elif self.rsw_id == 1:
            self.dio.reset(ch=1)
            self.dio.set_mode(mode="MD0", direction=0, equal=1, latch=0, ch=1)
        else:pass
        return

    def board_setting(self, z_mode=""):
        [self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=ch) 
                for ch in self.ch_list]
        return

    # for encoder
    def set_origin(self, req):
        self.origin = req.data
        self.origin_flag = True
        return

    # for dome_encoder
    def set_counter(self, counter, ch):
        self.counter["counter"] = counter.data
        self.counter["ch"] = ch
        self.counter_flag = True
        return

    def dio_function(self):
        while not rospy.is_shutdown():
            for ch, pub in zip(self.ch_list, self.pub):
                ret = self.dio.get_counter(unsigned=False, ch=ch)
                pub.publish(int(ret))

            if self.origin_flag:
                if self.origin == True:
                    self.board_setting(z_mode="CLS0")
                else:
                    self.board_setting(z_mode="")
                self.origin_flag = False
            else: pass

            if self.counter_flag:
                self.dio.set_counter(self.counter["counter"], ch=self.counter["ch"])
                self.counter_flag = False
            else: pass
            
            time.sleep(0.001)
            continue
        return


if __name__ == "__main__":
    rospy.init_node(name)
    cpz = CPZ6204()

    dio_thread = threading.Thread(
            target = cpz.dio_function,
            daemon = True,
        )
    dio_thread.start()

    rospy.spin()
