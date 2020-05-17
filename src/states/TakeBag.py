import rospy
from smach import State

class TakeBag(State):
	def __init__(self, tiago, follower):
		State.__init__(self, outcomes=['start_following', 'nothing_given'])
		self.tiago = tiago
		self.follower = follower

	def execute(self, userdata):


		self.tiago.play('reach_out_arm')

		self.tiago.talk('Please hand me your bag.')
		rospy.sleep(5)
		self.tiago.play('close_gripper')

		self.tiago.play('tuck_arm')
		self.tiago.talk('I will follow you now.')

		self.follower.start_following = True

		return 'start_following'
