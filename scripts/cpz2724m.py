#! /usr/bin/env python3

name = "cpz2724m"

#----
import sys
import time
import threading
import rospy
import std_msgs.msg
import pyinterface



class CPZ2724(object):

    ch_list = ["01","02","03","04","05","06","07","08","09","10",
                "11","12","13","14","15","16","17","18","19","20",
                "21","22","23","24","25","26","27","28","29","30",
                "31","32"]
    ch_list_byte = []
    ch_list_word = ["1_16, 17_32"]
    ch_list_dword = []
    
    def __init__(self):
        self.rsw_id = rospy.get_param("~rsw_id")

        self.pub = [rospy.Publisher(
                        name = "cpz2724_rsw{0}/di{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        latch = True,
                        queue_size = 1
                    ) for ch in self.ch_list]

        self.sub = [rospy.Subscriber(
                        name = "cpz2724_rsw{0}/do{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        callback = self.output_point,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list]

        self.sub_byte = [rospy.Subscriber(
                        name = "cpz2724_rsw{0}/do{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        callback = self.output_byte,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list_byte]

        self.sub_word = [rospy.Subscriber(
                        name = "cpz2724_rsw{0}/do{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        callback = self.output_word,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list_word]

        self.sub_dword = [rospy.Subscriber(
                        name = "cpz2724_rsw{0}/do{1}".format(self.rsw_id, ch),
                        data_class = std_msgs.msg.Bool,
                        callback = self.output_dword,
                        callback_args = ch,
                        queue_size = 1
                    ) for ch in self.ch_list_dword]

        try:
            self.dio = pyinterface.open(2724, self.rsw_id)
        except OSError as e:
            rospy.logerr(e, name, self.rsw_id)
            sys.exit()
        
        pass

    def output_point(self, req, ch):
        self.dio.output_point([req.data] ,int(ch))
        return

    def output_byte(self, req, ch):
        self.dio.output_byte("OUT{}".format(ch), req.data)
        return

    def output_word(self, req, ch):
        self.dio.output_word("OUT{}".format(ch), req.data)
        return

    def output_dword(self, req, ch):
        self.dio.output_dword("OUT{}".format(ch), req.data)
        return


    def pub_function(self):
        di_list = self.dio.input_dword()
        for ch, pub in zip(self.ch_list, self.pub):
            pub.publish(di_list[int(ch)-1])

        while not rospy.is_shutdown():
            ret = self.dio.input_dword() # ch1~32
            for ch, pub in zip(self.ch_list, self.pub):
                if ret[int(ch)-1] != di_list[int(ch)-1]:
                    pub.publish(ret[int(ch)-1])
                    di_list[int(ch)-1] = ret[int(ch)-1]
                else: pass

            time.sleep(0.001)
            continue
        
        return


if __name__ == "__main__":
    rospy.init_node(name)
    cpz = CPZ2724()

    pub_thread = threading.Thread(
            target = cpz.pub_function,
            daemon = True,
        )
    pub_thread.start()

    rospy.spin()
