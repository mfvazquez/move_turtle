#!/usr/bin/env python
import rospy
import numpy
import actionlib
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from tech_mvazquez.msg import * # Progress, Goal
from geometric_functions import *
from std_srvs.srv import *  # Empty


# CONSTANTS
NODE_NAME = 'turtle_movement'
QUEUE_SIZE = 10
RATE = 50.0
ACTION_SERVER_MOVE = 'move_action_server'
ACTION_SERVER_TELEPORT = 'teleport_action_server'
SPEED_PARAM = '/turtle/speed'
VEL_TOPIC = '/tech_mvazquez/turtle1/cmd_vel'
POSE_TOPIC = '/tech_mvazquez/turtle1/pose'
PAUSE_SERVICE = '/tech_mvazquez/pause'
RESUME_SERVICE = '/tech_mvazquez/resume'
TELEPORT_SERVICE = '/tech_mvazquez/teleport'
TELEPORT_SERVICE_TURTLESIM = '/tech_mvazquez/turtle1/teleport_absolute'


class Turtle:

    def __init__(self):
        self.rate = rospy.Rate(RATE)

        # TOPICS
        self.velocity_publisher = rospy.Publisher(VEL_TOPIC, Twist, queue_size=QUEUE_SIZE)
        self.pose_subscriber = rospy.Subscriber(POSE_TOPIC, Pose, self.get_pose)

        # Services
        self.turtlesim_teleport_service = rospy.ServiceProxy(TELEPORT_SERVICE_TURTLESIM, TeleportAbsolute)
        self.pause_service = rospy.Service(PAUSE_SERVICE, Empty, self.pause)
        self.resume_service = rospy.Service(RESUME_SERVICE, Empty, self.resume)
        # self.teleport_service = rospy.Service(TELEPORT_SERVICE, Goal, self.teleport)

        #Actions
        self.move_server = actionlib.SimpleActionServer(ACTION_SERVER_MOVE, ReachGoalAction, self.move, auto_start = False)
        self.move_server.start()
        self.teleport_server = actionlib.SimpleActionServer(ACTION_SERVER_TELEPORT, ReachGoalAction, self.teleport, auto_start = False)
        self.teleport_server.start()


        self.pose = None # Actual pose of the turtle
        self.paused = False # Flag to know if the turtle is paused
        self.goal = None # Goal that the turtle has to reach
        self.feedback = ReachGoalFeedback()
        self.status_canceled = False # Flag to know if the goal is canceled

        if not rospy.has_param(SPEED_PARAM):
            rospy.set_param(SPEED_PARAM,1)


    def get_pose(self, data):
        """ get the actual pose and update it from the pose_subscriber """
        self.pose = data


    def pause(self, data):
        """ Pause the turtle, it can continue the current
        goal if resume is called """
        self.paused = True
        return EmptyResponse()


    def resume(self, data):
        """ Continue with the current goal """
        self.paused = False
        return EmptyResponse()


    def stop(self):
        """ Stop the turtle """
        vel_message = Twist()
        vel_message.linear.x = 0
        vel_message.angular.z = 0
        self.velocity_publisher.publish(vel_message)


    def rotate(self, orientation):
        """ rotate the turtle until it reach the orientation angle.
        Pre: orientation must be in radians"""

        vel_message = Twist()
        current_speed = rospy.get_param(SPEED_PARAM)
        current_tolerance = current_speed/RATE
        current_angular_distance = normalize_angle(orientation - self.pose.theta)

        while abs(current_angular_distance) >= current_tolerance:

            self.check_preempt_request() # Check if the goal is canceled
            if self.status_canceled:
                self.stop()
                return

            while self.paused: # Check if a paused was called
                self.stop()
                self.send_new_status('pause', None)
                self.rate.sleep()

            vel_message.angular.z = current_speed * numpy.sign(current_angular_distance)
            self.velocity_publisher.publish(vel_message)

            self.rate.sleep()

            current_speed = rospy.get_param(SPEED_PARAM)
            current_tolerance = current_speed/RATE
            current_angular_distance = normalize_angle(orientation - self.pose.theta)

        self.stop()


    def go_to_objective(self):
        """ move the turtle until it reach the objective pose self.goal"""

        vel_message = Twist()

        current_distance = distance(self.pose, self.goal)
        current_angular_distance = angle_between_poses(self.pose, self.goal)
        current_speed = rospy.get_param(SPEED_PARAM)
        current_tolerance = current_speed/RATE
        initial_distance = current_distance
        while abs(current_distance) >= current_tolerance:

            self.check_preempt_request() # check if the goal is canceled
            if self.status_canceled:
                self.stop()
                return

            progress = (1 - current_distance/initial_distance)*100
            self.send_new_status('moving', progress)

            while self.paused: # check if the turtle must be paused
                self.stop()
                self.send_new_status('pause', None)
                self.rate.sleep()

            vel_message.linear.x = current_speed * numpy.sign(current_distance)
            vel_message.angular.z = current_speed * numpy.sign(current_angular_distance)
            self.velocity_publisher.publish(vel_message)

            self.rate.sleep()

            current_distance = distance(self.pose, self.goal)
            current_angular_distance = angle_between_poses(self.pose, self.goal)
            current_speed = rospy.get_param(SPEED_PARAM)
            current_tolerance = current_speed/RATE

        self.send_new_status('goal reached', 100)

        self.stop()


    def teleport(self, goal):
        """ teleport the turtle to the goal provided. It use
        the services teleport_absolute of turtlesim_node """
        self.turtlesim_teleport_service(goal.x, goal.y, goal.theta)
        self.teleport_server.set_succeeded(ReachGoalResult())


    def find_goal(self):
        """ rotate the turtle until it point to the goal """
        if not self.is_near_goal():
            angular_distance = angle_between_points(self.pose, self.goal)
            self.rotate(angular_distance)
            

    def is_near_goal(self):
        """ Check if the turtle is near the goal.
        It consider near if the error is lower than the
        tolerance calculated based on the speed and rate of
        the turtle pose.
        Pos: Return True if it is near, false otherwise"""

        current_distance = distance(self.pose, self.goal)
        current_speed = rospy.get_param(SPEED_PARAM)
        current_tolerance = current_speed/RATE
        if abs(current_distance) >= current_tolerance: # It is not near
            return False
        return True


    def move(self, goal):
        """ This method consist on 3 steps, this method is
        used by the action_server_move:
            1- rotate the turtle until it points to the Goal
            2- move the turtle to the goal
            3- rotate until it has the orientation required """

        self.status_canceled = False
        while not self.pose:         # Wait a pose
            self.rate.sleep()

        self.goal = goal

        # Point to the Goal
        self.send_new_status('finding goal', 0)
        self.find_goal()
        if self.status_canceled:
            return

        # Move to the goal
        self.go_to_objective()
        if self.status_canceled:
            return

        self.send_new_status('reaching orientation', None)
        self.rotate(self.goal.theta)
        if self.status_canceled:
            return

        # Rotate to the orientation requiered
        self.send_new_status('waiting for a goal', None)
        self.rotate(self.goal.theta)
        if self.status_canceled:
            return

        self.move_server.set_succeeded(ReachGoalResult())


    def check_preempt_request(self):
        """ check if the goal is canceled by the client """
        if self.move_server.is_preempt_requested():
            self.move_server.set_preempted()
            self.status_canceled = True


    def send_new_status(self, status, progress):
        """ Send a status to the clients of the actionlib action_server_move """
        if progress is not None:
            self.feedback.distance_progress = progress
        self.feedback.status = status
        self.move_server.publish_feedback(self.feedback)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        server = Turtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
