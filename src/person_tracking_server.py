#!/usr/bin/env python2
import sys
# sys.path.insert(0, '/home/praveen/Desktop/project/tiago-lib/tiago_ws/src/lasr_object_detection_yolo/opencv4/python')
print(sys.version)

import roslib
import rospy
import actionlib
from cv_bridge import CvBridge, CvBridgeError

from face_detector_tracker import FaceDetectorTracker
from jeff_object_tracking.msg import TrackedObject, ObjectTrackingAction, ObjectTrackingFeedback
from sensor_msgs.msg import Image
import cv2

class ObjectTrackingServer:
    _feedback = ObjectTrackingFeedback()

    def __init__(self):
        self.bridge = CvBridge()
        self._as = actionlib.SimpleActionServer('object_tracking', ObjectTrackingAction, self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        self.face_detector_tracker = FaceDetectorTracker(   \
            goal.yolo_confidence, goal.yolo_nms,             \
            goal.object_list, goal.redetect_rate,           \
            goal.face_detect_confidence)

        print('object tracking goal : ', goal)
        image_sub = rospy.Subscriber(goal.input_topic, Image, self.tracking_cb)

        rospy.spin()

    def tracking_cb(self, data):
        print('inside tracking cb with image ')
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError:
            return

        print('inside tracking image callback')
        # TODO: if time gap is too long, delete all the prev tracked objects, here
        self.face_detector_tracker.update(frame)

        tracked_objects = []
        print('trackers from yolo : ', self.face_detector_tracker.trackers)
        for tracker in self.face_detector_tracker.trackers:
            tracked_objects.append(TrackedObject(tracker.id, tracker.label, tracker.box, tracker.name))
            # draw bounding box
            (x, y, w, h) = [int(v) for v in tracker.box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            name_text = "name: {}".format(tracker.name)
            id_text = "id: {}, {}".format(tracker.id, tracker.label)
            cv2.putText(frame, name_text, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, id_text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # display bounding-boxed image
        cv2.imshow('person tracking server', frame)
        cv2.waitKey(1)

        self._feedback.tracked_objects = tracked_objects
        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('object_tracking_server')
    object_tracking_server = ObjectTrackingServer()
    rospy.spin()
