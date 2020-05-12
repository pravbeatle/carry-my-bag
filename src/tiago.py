import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import String

import yaml
import numpy as np
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class Tiago:
    def __init__(self):

        self.goal_sent = False	# if goal is sent, we need to cancel before shutting down

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(10)

		# set linear and angular velocities
        self.velocity = Twist()

		# publish and subcribe to relevant topics
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait until the action server comes up")
        self.move_base.wait_for_server(rospy.Duration(5))

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)


    def move(self, linear=(0,0,0), angular=(0,0,0)):

        self.velocity.linear.x = linear[0] 	# Forward or Backward with in m/sec.
        self.velocity.linear.y = linear[1]
        self.velocity.linear.z = linear[2]

        self.velocity.angular.x = angular[0]
        self.velocity.angular.y = angular[1]
        self.velocity.angular.z = angular[2] 	# Anti-clockwise/clockwise in radians per sec

        self.velocity_publisher.publish(self.velocity)


    def calculate_pos_quat(self, x, y):
        theta = 0.1
        pos = {'x': x, 'y' : y, 'z': 0}
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        return pos, quat

    # takes a pose/quat or an x, y which is then converted
    def go_to(self, pose, quat, mode=''):

        if mode == 'point':
            pose, quat = self.calculate_pos_quat(pose, quat)

        # send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pose['x'], pose['y'], pose['z']),
        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)

        rospy.loginfo('Sent goal and waiting for robot to carry it out...')
        success = self.move_base.wait_for_result(rospy.Duration(90))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

            self.goal_sent = False

        return result


    def talk(self, speech_in):
        # Create the TTS goal and send it
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = speech_in
        self.tts_client.send_goal(tts_goal)


    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
