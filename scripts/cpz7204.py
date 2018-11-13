#! /usr/bin/env python3

name = "cpz7204"

#----
import sys
import time
import threading
import rospy
import std_msgs.msg
import pyinterface


class CPZ7204(object):

    data = None
    flag = False

    def __init__(self):
        self.rsw_id = rospy.get_param("~rsw_id")

        self.pub_busy = rospy.Publisher(
                        name = "/cpz7204_rsw{0}/busy".format(self.rsw_id),
                        data_class = std_msgs.msg.Bool,
                        latch = True,
                        queue_size = 1,
                    )

        self.pub_pEL = rospy.Publisher(
                        name = "/cpz7204_rsw{0}/p_EL".format(self.rsw_id),
                        data_class = std_msgs.msg.Bool,
                        latch = True,
                        queue_size = 1,
                    )

        self.pub_mEL = rospy.Publisher(
                        name = "/cpz7204_rsw{0}/m_EL".format(self.rsw_id),
                        data_class = std_msgs.msg.Bool,
                        latch = True,
                        queue_size = 1,
                    )

        self.sub = rospy.Subscriber(
                        name = "/cpz7204_rsw{0}/step".format(self.rsw_id),
                        data_class = std_msgs.msg.Bool,
                        callback = self.set_function,
                        queue_size = 1, 
                    )


        try:
            self.mot = pyinterface.open(7204, self.rsw_id)
            self.mot.initialize()
            self.mot.set_limit_config('LOGIC', '+EL -EL', axis=1)
        except OSError as e:
            rospy.logerr(e, name, self.rsw_id)
            sys.exit()

        pass

    def set_function(self, req):
        self.data = req.data
        self.flag = True
        return

    def move_function(self):
        while not rospy.is_shutdown():
            if not self.flag:
                time.sleep(0.01)
                continue

            else:
                if self.data: # NASCO
                    self.mot.set_motion(mode="JOG", step = -1)
                else: # SMART
                    self.mot.set_motion(mode="JOG", step = 1)
                self.mot.start_motion(mode="JOG")
                self.flag = False

        return

    def pub_function(self):
        # init
        status_last = self.mot.get_status()
        self.pub_busy.publish(status["busy"])
        self.pub_pEL.publish(status["limit"]["+EL"])
        self.pub_mEL.publish(status["limit"]["-EL"])

        while not rospy.is_shutdown():
            status = self.mot.get_status()

            if status != status_last:
                if status["busy"] != status_last["busy"]:
                    self.pub_busy.publish(status["busy"])
                elif status["limit"]["+EL"] != status_last["limit"]["+EL"]:
                    self.pub_pEL.publish(status["limit"]["+EL"])
                elif status["limit"]["-EL"] != status_last["limit"]["-EL"]:
                    self.pub_mEL.publish(status["limit"]["-EL"])
                else: pass
                status_last = status
            else: pass
            
            time.sleep(0.01)
            continue
        
        return


if __name__ == "__main__":
    rospy.init_node(name)
    cpz = CPZ7204()

    pub_thread = threading.Thread(
            target = cpz.pub_function,
            daemon = True,
        )
    move_thread = threading.Thread(
            target = cpz.move_function,
            daemon = True,
        )
    pub_thread.start()
    move_thread.start()

    rospy.spin()
