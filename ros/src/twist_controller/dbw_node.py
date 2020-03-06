#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

PUBLISHING_RATE = 50
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class VehicleParams(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        self.total_vehicle_mass = None


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        carla_params = VehicleParams()
        carla_params.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        carla_params.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        carla_params.brake_deadband = rospy.get_param('~brake_deadband', .1)
        carla_params.decel_limit = rospy.get_param('~decel_limit', -5)
        carla_params.accel_limit = rospy.get_param('~accel_limit', 1.)
        carla_params.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        carla_params.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        carla_params.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        carla_params.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        carla_params.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        carla_params.total_vehicle_mass = carla_params.vehicle_mass + carla_params.fuel_capacity * GAS_DENSITY

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        # set queue_size=1 for both publisher and subscriber to only handle the newest message

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        self.controller = Controller(vehicle_params=carla_params)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.pose = None
        self.final_waypoints_2d = None
        self.throttle = self.steering = self.brake = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)  # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)

            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                # cte = self.get_cte(self.final_waypoints_2d, self.pose)
                cte =0
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel, cte)

            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def final_waypoints_cb(self, msg):
        final_waypoints = msg.waypoints
        self.final_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                   final_waypoints]
        pass

    def pose_cb(self, msg):
        self.pose = msg
        pass

    # def get_cte(self, final_waypoints_2d, current_pose):
    #     if final_waypoints_2d is not None and current_pose is not None:
    #         # convert from quaternion to yaw
    #         orientation_q = current_pose.pose.orientation
    #         _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    #
    #         # get the x and y of the current pose
    #         x = current_pose.pose.position.x
    #         y = current_pose.pose.position.y
    #         x_transform = []
    #         y_transform = []
    #         cos_yaw = math.cos(-yaw)
    #         sin_yaw = math.sin(-yaw)
    #
    #         # set the number of points to fit
    #         points_to_fit = 10
    #         points_to_fit = min(points_to_fit, len(final_waypoints_2d))
    #
    #         # transform the waypoints to the vehicle frame
    #         for i in range(points_to_fit):
    #             waypoint_x, waypoint_y = final_waypoints_2d[i]
    #             x_d = waypoint_x - x
    #             y_d = waypoint_y - y
    #             x_transform.append(x_d * cos_yaw - y_d * sin_yaw)
    #             y_transform.append(x_d * sin_yaw + y_d * cos_yaw)
    #
    #         # Fit a 3rd degree polynomial to the waypoints
    #         degree = 3
    #         coefficients = np.polyfit(x_transform, y_transform, degree)
    #
    #         return np.polyval(coefficients, 0.0)
    #     else:
    #         return 0

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
