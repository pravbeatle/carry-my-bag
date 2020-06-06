import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import String

import speech_recognition as sr
import yaml
import numpy as np
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx

class Tiago:
    def __init__(self):

        self.move_base_goal_sent = False	# if goal is sent, we need to cancel before shutting down
        self.play_motion_goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(10)

		# set linear and angular velocities
        self.velocity = Twist()

		# publish and subcribe to relevant topics
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait until the move_base action server comes up")
        self.move_base.wait_for_server(rospy.Duration(5))

        self.play_motion = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        rospy.loginfo("Wait until the play_motion action server comes up")
        self.play_motion.wait_for_server(rospy.Duration(5))

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
        self.move_base_goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pose['x'], pose['y'], pose['z']),
        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)

        rospy.loginfo('Sent move_base goal and waiting for robot to carry it out...')
        success = self.move_base.wait_for_result(rospy.Duration(90))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()
            self.move_base_goal_sent = False

        return result


    def play(self, motion_name):

        self.play_motion_goal_sent = True
        play_goal = PlayMotionGoal()
        play_goal.motion_name = motion_name
        play_goal.skip_planning = True

        self.play_motion.send_goal(play_goal)
        rospy.loginfo('Sent play_motion goal and waiting for robot to carry it out... ')
        success = self.play_motion.wait_for_result(rospy.Duration(30))

        state = self.play_motion.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.play_motion.cancel_goal()
            self.play_motion_goal_sent = False

        return result


    def recognize_speech(self, duration=None):
        # transcribe speech from recorded from microphone
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        # adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        try:
            response["transcription"] = recognizer.recognize_google(audio)
        except sr.RequestError:
            # API was unreachable or unresponsive
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            # speech was unintelligible
            response["success"] = False
            response["error"] = "Unable to recognize speech"

        return response


    def talk(self, speech_in):
        # Create the TTS goal and send it
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
        speech_engine = pyttsx.init()
        speech_engine.say(speech_in)
        speech_engine.runAndWait()

        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = speech_in
        self.tts_client.send_goal(tts_goal)



    def shutdown(self):
        if self.move_base_goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
