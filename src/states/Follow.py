import rospy
from smach import State

class Follow(State):
    def __init__(self, tiago, follower):
        State.__init__(self, outcomes=['follow_done', 'still_following'])
        self.tiago = tiago
        self.follower = follower

    def execute(self, userdata):

        speech = self.tiago.recognize_speech()
        print('TRANSCRIPTED SPEECH : ', speech)
        if speech["transcription"] and ('please' in speech['transcription'].encode('ascii','ignore').split()):
            self.follower.start_following = False
            self.tiago.talk('Please take your bag.')
            self.tiago.play('reach_out_arm')
            self.tiago.play('open_gripper')
            rospy.sleep(3)

            self.tiago.talk('I will stop following you now. Thank You.')
            self.tiago.play('home')
            self.tiago.go_to_conf_room()

            return 'follow_done'
        else:
            return 'still_following'
