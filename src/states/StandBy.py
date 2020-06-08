import rospy
from smach import State

class StandBy(State):
    def __init__(self, tiago, follower):
        State.__init__(self, outcomes=['porter_request', 'loop_back'])
        self.tiago = tiago
        self.follower = follower

    def execute(self, userdata):

        print(self.follower.target_reached_first_time)
        self.follower.start_following = True
        while not self.follower.target_reached_first_time:
            continue

        self.follower.start_following = False
        self.tiago.talk('Hello ' + self.follower.target + ', Do you need help with your bag ?')

        speech = self.tiago.recognize_speech()
        print('TRANSCRIPTED SPEECH : ', speech)
        if speech["transcription"] and ('yes' or 'please' in speech['transcription'].encode('ascii','ignore').split()):
            return 'porter_request'
        else:
            return 'loop_back'
