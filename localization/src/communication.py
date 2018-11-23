import rospy
import numpy as np

from motor_movement import to_motor, to_eucl
from core.msg import Command

class Communication(object):
    """Python class for constructing and sending strings for serial"""
    def __init__(self):
        self.pub = rospy.Publisher('commands', Command, queue_size=10)

    def send_cmd(self, cmd):
        msg_txt = (cmd + "\n")
        msg = Command()
        msg.command = msg_txt
        # print(msg)
        self.pub.publish(msg)


    def set_motor(self, eucl):
        res = to_motor(np.matrix(eucl))
        speeds = np.round(res).flatten().tolist()[0]
        str_speeds = [str(int(s)) for s in speeds]
        str_ = "sd:" + ":".join(str_speeds)
        # print("speeds: " + str_)
        self.send_cmd(str_)


    def set_pid(self, p, i, d):
        str_pid = [str(x) for x in (p,i,d)]
        self.send_cmd("pid:" + ":".join(str_pid))

        
    def start_timers(self, send_func, stop_func):
        self.timer_cb = send_func
        self.sender = rospy.Timer(rospy.Duration.from_sec(0.02), send_func)
        self.stopper = rospy.Timer(rospy.Duration(2), self.get_stop_cb(stop_func), oneshot=True)
        

    def get_stop_cb(self,stop_func):
        def r(e):
            stop_func(e)
            self.sender.shutdown()
        return r


    