#!/usr/bin/env python
# -*- coding: utf-8 -*-
# sys.path.extend(['/home/robot/catkin_ws/devel/lib/python2.7/dist-packages', '/opt/ros/kinetic/lib/python2.7/dist-packages', '/usr/lib/python2.7'])
from __future__ import print_function

import smach
import rospy


class Foo(smach.State):
    def __init__(self, outcomes=['o1', 'o2']):
        smach.State.__init__(self, outcomes)
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Executing FOO")
        if True:
            return 'o1'
        else:
            return 'o2'


class Bar(smach.State):
    def __init__(self, outcomes=['o2']):
        smach.State.__init__(self, outcomes)

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'o2'


if __name__ == '__main__':
    rospy.init_node('state_machine0')

    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    with sm:
        smach.StateMachine.add('FOO', Foo(), transitions={
            'o1': 'BAR',
            'o2': 'outcome4'
        })
        smach.StateMachine.add('BAR', Bar(), transitions={
            'o2': 'FOO'
        })

        outcome = sm.execute()
