#!/usr/bin/env python2
import rospy
from tiago import Tiago
from person_follower import PersonFollower
from smach import StateMachine
from states.TakeBag import TakeBag
from states.StandBy import StandBy

if __name__ == '__main__':
    rospy.init_node('porter')
    tiago = Tiago()
    follower = PersonFollower('')

    sm = StateMachine(outcomes=['success', 'failure'])  # the end states of the machine
    with sm:

        StateMachine.add('stand_by', StandBy(tiago, follower), transitions={'porter_request': 'take_bag', 'loop_back': 'stand_by'})

        StateMachine.add('take_bag', TakeBag(tiago, follower), transitions={'start_following': 'success', 'nothing_given': 'take_bag'})


        sm.execute()

    rospy.spin()
