import rospy
from smach import State

class TakeBag(State):
	def __init__(self, tiago):
		State.__init__(self, outcomes=['start_following', 'nothing_given'])
		self.tiago = tiago

	def execute(self, userdata):


		self.tiago.play('unfold_arm')
		self.tiago.talk('Please hand me your bag.')
		rospy.sleep(5)
		self.tiago.play('home')
		self.tiago.talk('I will follow you now.')

		return 'start_following'
