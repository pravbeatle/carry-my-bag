import rospy
from smach import State

class StandBy(State):
    def __init__(self, tiago, follower):
        State.__init__(self, outcomes=['porter_request', 'loop_back'])
        self.tiago = tiago
        self.follower = follower

    def execute(self, userdata):

        print(self.follower.target)
        while self.follower.last_followed != self.follower.target:
            continue

        self.tiago.talk('Hello ' + self.follower.target + ', Do you need help with your bag ?')
        print('ASSUME AFFIRMATIVE')
        rospy.sleep(2)

        return 'porter_request'
