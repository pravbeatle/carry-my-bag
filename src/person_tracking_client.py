#!/usr/bin/env python2
import rospy
from tiago import Tiago
from person_follower import PersonFollower

from smach import StateMachine
from states.TakeBag import TakeBag


if __name__ == '__main__':
    rospy.init_node('porter')
    tiago = Tiago()
    follower = PersonFollower('jess', tiago)

    sm = StateMachine(outcomes=['success', 'failure'])  # the end states of the machine
    with sm:
        StateMachine.add('take_bag', TakeBag(tiago), transitions={'start_following': 'success', 'nothing_given': 'failure'})


        # sm.execute()




    rospy.spin()
