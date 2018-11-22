#!/usr/bin/env python
from __future__ import print_function
from collections import defaultdict
from itertools import product

import rospy
import numpy as np

from communication import Communication
from serial.msg import WheelSpeed

from motor_movement import slippage


comm = Communication()

err = []
last_best = 1000000
is_measuring = False

def callback(data):
    """Handle subscriber data."""
    global err, is_measuring
    # rospy.loginfo("w:\t%d\t%d\t%d\t%d", data.wheel1, data.wheel2, data.wheel3, data.wheel4)

    if is_measuring:
        speeds = np.array([data.wheel1, data.wheel2, data.wheel3, data.wheel4])
        err.append(np.sum(slippage(speeds) ** 2))

def start():
    # Create a subscriber for wheelspeeds
    rospy.Subscriber('wheelspeed', WheelSpeed, callback)
    comm.send_cmd("gs")
    next_batch()

tests = []
batch_tests = []
results = defaultdict(list)

cur = [(768, 768, 256), (640, 128, 128)]
learning_rate = 64 # lower learning rate 4 times after each batch

# 10 sec per combination
# 4 values per learning rate
# => 81 tests per learning rate
# => 810 seconds
# 3 learning rates = [256, 64, 16]
# => 40 minutes
# ==> need saving of state

#  (768, 768, 256), (640, 128, 128)
# (832, 832, 128), (704, 640, 128)
# (768, 768, 64), (864, 768, 64)
# (872, 776, 72)
#(864, 768, 64), (864, 780, 64)
# (832, 748, 80), (832, 796, 80), (832, 780, 64), (880, 748, 32)


def get_batch_tests(last_bests=[(513, 512, 512)], learning_rate=256):
    "Returns all of the tests of the current batch"
    bottom_tests = [tuple(map(lambda x: x-(2*learning_rate), last_best)) for last_best in last_bests]

    changes = [ change for change in range(learning_rate, learning_rate*4, learning_rate) for _ in range(6)]

    def make_change(t, selector, x):
        l = list(t)
        for i,v in enumerate(selector):
            if v:
                l[i] += x
        return tuple(l)
    
    all_tests = []
    for bottom_test in bottom_tests:
        for i in product([True, False], repeat=3):
            for change in changes:
                all_tests.append(make_change(bottom_test, i, change))


    return all_tests
    #return [(768, 768, 64), (768, 768, 64), (864, 768, 64), (864, 768, 64)]


def next_batch():
    global cur, tests, batch_tests

    # batch_tests = get_batch_tests(cur, learning_rate)
    batch_tests = [(352, 268, 64), (352, 268, 64), (352, 100, 64), (352, 100, 64), (352, 258, 64), (352, 258, 64)]
    
    tests = batch_tests[:]
    # start testing
    next_test()

def finish_batch():
    global batch_tests, results, cur, learning_rate
    # choose least err
    sorted_results = sorted(batch_tests, key=lambda test: np.sum(np.sum(results[test])))
    # cur = min(batch_tests, key=lambda test: np.mean(np.sum(results[test])))
    cur = sorted_results[:4:2]
    print("least err: ", cur)
    print("results: ", results)
    results = defaultdict(list)

    # update learning rate
    learning_rate /= 4
    print("------------------------")
    print()
    print("new learning rate:\t", learning_rate)

    next_batch()
    
    

def next_test():
    global tests, cur, is_measuring
    cur = tests.pop()
    comm.send_cmd("gs")
    is_measuring = True
    #print(cur)
    test_constants(*cur)


dir = -1
def next_dir():
    global dir
    dir *= -1
    return dir

def test_constants(p, i, d):
    comm.set_pid(p, i, d)
    move_forward(next_dir())

def stop_func(_):
    global err, is_measuring, last_best
    err = err[1:]
    is_measuring = False
    sum_err = sum(err)
    print("finished test: ", cur)
    print("sum err: ", sum_err)
    print("worse than best: ", sum_err - last_best)
    if (sum_err - last_best < 0):
        last_best = sum_err
    comm.set_motor([[0],[0],[0]])
    

    results[cur].append(err)
    err = []

    # start the next test after 1 seconds, (if there is a next test)
    if tests:
        rospy.Timer(rospy.Duration(4), lambda e: next_test(), oneshot=True)
    else:
        finish_batch()

def move_forward(spd):
    eucl = np.matrix("1; 0; 0") * spd
    
    # start the timers 
    comm.start_timers(
        send_func = lambda _: comm.set_motor(eucl),
        # send_func = lambda _: print("sender called"),
        # stop_func = lambda _: comm.set_motor([[0],[0],[0]])
        stop_func = stop_func
    )

def shutdown():
    print("results: \n", results)


if __name__ == '__main__':

    rospy.init_node('testPID')
    rospy.loginfo("starting testPID")
    rospy.on_shutdown(shutdown)

    try:
        start()
    except rospy.ROSInterruptException:
        print("quitting")
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()