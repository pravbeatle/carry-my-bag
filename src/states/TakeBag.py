import rospy
from smach import State

class TakeBag(State):
	def __init__(self, tiago, follower):
		State.__init__(self, outcomes=['start_following', 'nothing_given'])
		self.tiago = tiago
		self.follower = follower

	def execute(self, userdata):

		if not self.tiago.arm_reached:
			self.tiago.play('reach_out_arm')
			self.tiago.arm_reached = True

		self.tiago.talk('Please hand me your bag.')

		speech = self.tiago.recognize_speech()
		print('TRANSCRIPTED SPEECH : ', speech)
		if speech["transcription"] and ('thank' in speech['transcription'].encode('ascii','ignore').split()):
			self.tiago.play('close_gripper')
			self.tiago.play('tuck_arm')

			self.tiago.talk('I will follow you now.')
			self.follower.start_following = True

			return 'start_following'
		else:
			return 'nothing_given'
