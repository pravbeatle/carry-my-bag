
from smach import State

class TakeBag(State):
	def __init__(self, tiago):
		State.__init__(self, outcomes=['nothing_given'])
		self.tiago = tiago

	def execute(self, userdata):

		


		self.tiago.talk('Please hand me your bag.')

		return 'nothing_given'
