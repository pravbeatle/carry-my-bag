#!/usr/bin/env python2

import message_filters
import actionlib

import numpy as np
import tf
from tf2_ros import TransformException
from math import sqrt, atan2

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped, Vector3, PoseWithCovarianceStamped, Quaternion
from control_msgs.msg import PointHeadAction, PointHeadGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from jeff_object_tracking.msg import ObjectTrackingAction, ObjectTrackingGoal, ObjectTrackingFeedback
from Queue import Queue, Empty



class PersonFollower:
    INPUT_TOPIC = 'image_rect_color'
    TARGET_DIST = 1.5
    MOVE_BASE_RESET = 2

    def __init__(self, target):
        self.target = target
        self.last_followed = None
        self.time_last = rospy.Time.now()

        self.transform_listener = tf.TransformListener()

        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # point head client
        self.point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.point_head_client.wait_for_server()

        # object tracking client
        self.client = actionlib.SimpleActionClient('object_tracking', ObjectTrackingAction)
        self.client.wait_for_server()

        # create and send tracking goal
        goal = ObjectTrackingGoal(                          \
            input_topic = self.INPUT_TOPIC,                 \
            yolo_confidence = 0.5, yolo_nms = 0.3,          \
            object_list = ('person',), redetect_rate = 15,  \
            face_detect_confidence = 0.5)
        self.client.send_goal(goal)

        # camera output redirection for processing
        self.redirect_pub = rospy.Publisher(self.INPUT_TOPIC, Image, queue_size = 1)

        # subscriber and queue to retrieve processed result
        self.tracking_sub = rospy.Subscriber('object_tracking/feedback', ObjectTrackingFeedback, self.feedback_in)
        self.feedback_queue = Queue()

        # camera subscribers for redirection
        image_sub = message_filters.Subscriber('/xtion/rgb/image_rect_color', Image)
        pcl2_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl2_sub], 2, 0.2)
        ts.registerCallback(self.follow_person)

    def follow_person(self, image, pcl):
        print('entered callback')
        self.redirect_pub.publish(image)
        print('post publishing image')
        data = self.feedback_queue.get()

        print('feedback from server : ', data)
        for tracked_object in data.feedback.tracked_objects:
            if tracked_object.name == self.target:
                cloud = np.fromstring(pcl.data, np.float32)
                # get centre point x,y,z
                centre_x = int(tracked_object.box[0] + tracked_object.box[2]/2)
                centre_y = int(tracked_object.box[1] + tracked_object.box[3]/2)
                centre_index = (centre_y * pcl.width * 8) + (centre_x * 8)
                (x, y, z) = cloud[centre_index : centre_index + 3]
                # if nan, dont do anything
                if np.isnan([x, y, z]).any():
                    break
                centre_point = Point(x, y, z)
                # create head goal
                ph_goal = PointHeadGoal()
                ph_goal.target.header.frame_id = 'xtion_optical_frame'
                ph_goal.max_velocity = 1
                ph_goal.min_duration = rospy.Duration(0.5)
                ph_goal.target.header.stamp = rospy.Time(0)
                ph_goal.target.point = centre_point
                ph_goal.pointing_frame = 'head_2_link'
                ph_goal.pointing_axis = Vector3(1,0,0)

                self.point_head_client.send_goal(ph_goal)

                # greet the person if not yet followed
                if not tracked_object.name == self.last_followed:
                    speech_out = 'Hello ' + tracked_object.name + ', I will follow you.'
                    self.talk(speech_out)
                    self.last_followed = tracked_object.name

                if (rospy.Time.now() - self.time_last).secs >= self.MOVE_BASE_RESET:
                    self.time_last = rospy.Time.now()
                    try:
                        self.transform_listener.waitForTransform(pcl.header.frame_id, 'map', pcl.header.stamp, rospy.Duration(4.0))
                        centre_point_stamped = PointStamped(pcl.header, centre_point)
                        person_point_stamped = self.transform_listener.transformPoint('map', centre_point_stamped)
                        person_point = person_point_stamped.point

                        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
                        robot_point = amcl_msg.pose.pose.position

                        dist_x = person_point.x - robot_point.x
                        dist_y = person_point.y - robot_point.y
                        euclidian_dist = sqrt(dist_x * dist_x + dist_y * dist_y)

                        # calculate target point if euclidian distance is not within threshold.
                        # otherwise tiago is nearby, rotate around current point.
                        if euclidian_dist > self.TARGET_DIST + (self.TARGET_DIST/10):
                            # ratio of (desired dist)/(total dist)
                            ratio = (euclidian_dist - self.TARGET_DIST)/euclidian_dist
                            # add (ratio * actual dist) to robot point, basically scale the triangle
                            target_x = robot_point.x + (ratio * dist_x)
                            target_y = robot_point.y + (ratio * dist_y)
                        else:
                            target_x = robot_point.x
                            target_y = robot_point.y

                        target_point = Point(target_x, target_y, 0)

                        # since point is along same line, use current pos to get new rotation
                        current_angle = atan2(dist_y, dist_x)
                        (x, y, z, w) = tf.transformations.quaternion_from_euler(0, 0, current_angle)
                        target_quaternion = Quaternion(x, y, z, w)

                        # create and send move base goal
                        mb_goal = MoveBaseGoal()
                        mb_goal.target_pose.header.frame_id = 'map'
                        mb_goal.target_pose.header.stamp = rospy.Time.now()
                        mb_goal.target_pose.pose.position = target_point
                        mb_goal.target_pose.pose.orientation = target_quaternion

                        self.move_base_client.send_goal(mb_goal)
                        print 'going to person'
                    except TransformException as e:
                        print 'transform error! probably tf extrapolation'

                break

    def feedback_in(self, data):
        print('inside feedback cb')
        self.feedback_queue.put(data)
