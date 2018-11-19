#!/usr/bin/env python
from __future__ import print_function
from collections import defaultdict

import rospy
import numpy as np

from communication import Communication
from serial.msg import WheelSpeed

from motor_movement import slippage

# hello test
comm = Communication()

err = []

def callback(data):
    """Handle subscriber data."""
    global err
    # rospy.loginfo("w:\t%d\t%d\t%d\t%d", data.wheel1, data.wheel2, data.wheel3, data.wheel4)
    speeds = np.array([data.wheel1, data.wheel2, data.wheel3, data.wheel4])

    err.append(np.sum(slippage(speeds) ** 2))

def start():
    # Create a subscriber for wheelspeeds
    rospy.Subscriber('wheelspeed', WheelSpeed, callback)
    comm.send_cmd("gs")
    test_next()

tests_lst = [(64,0,128), (64,0,512), (64,0,64), (256,32,128), (256,64,128), (256,128,128), (256,256,128), (256,64,256), (512,64,512), (256,64,1024)]
# repeat each test 4 times
tests = [test for test in tests_lst for _ in range(4)]
results = defaultdict(list)

cur = ()
def test_next():
    global tests, cur
    cur = tests.pop()
    test_constants(*cur)


dir = -1
def next_dir():
    global dir
    dir *= -1
    return dir

def test_constants(p, i, d):
    """ Returns squared error"""
    # todo
    comm.set_pid(p, i, d)
    move_forward(next_dir() )

def stop_func(_):
    global err
    print("finished test: ", cur)
    print("sum err: ", sum(err))
    comm.set_motor([[0],[0],[0]])

    results[cur].append(err)
    err = []

    # start the next test after 1 seconds, (if there is a next test)
    if tests:
        rospy.Timer(rospy.Duration(2), lambda e: test_next(), oneshot=True)
    else:
        print(results)

def move_forward(spd):
    eucl = np.matrix("1; 0; 0") * spd
    
    # start the timers 
    comm.start_timers(
        send_func = lambda _: comm.set_motor(eucl),
        # send_func = lambda _: print("sender called"),
        # stop_func = lambda _: comm.set_motor([[0],[0],[0]])
        stop_func = stop_func
    )


if __name__ == '__main__':

    rospy.init_node('testPID')
    rospy.loginfo("starting testPID")
    start()
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()