import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

MAMAX_TORQUE = 400


class Controller(object):
    def __init__(self, vehicle_params):

        # TODO: Implement
        self.yaw_controller = YawController(
            wheel_base=vehicle_params.wheel_base,
            steer_ratio=vehicle_params.steer_ratio,
            min_speed=0.1,
            max_lat_accel=vehicle_params.max_lat_accel,
            max_steer_angle=vehicle_params.max_steer_angle)

        self.vehicle_params = vehicle_params
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0., mn=0,
                                       mx=0.30 * (self.vehicle_params.accel_limit))

        self.cte_controller = PID(kp=0.5, ki=0., kd=0.02, mn=-self.vehicle_params.max_steer_angle,
                                  mx=self.vehicle_params.max_steer_angle)

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = .02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # rospy.logwarn("DBW Status: {0}".format(dbw_enabled))

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.cte_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        # sample time calculation
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # steering calculation
        steering_base = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering_cte = self.cte_controller.step(cte, sample_time)
        steering_total = steering_base + steering_cte
        steering = max(min(self.vehicle_params.max_steer_angle, steering_total), -self.vehicle_params.max_steer_angle)

        # throttle calculation
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        throttle = self.throttle_controller.step(vel_error, sample_time)

        # brake calculation
        brake = 0
        if linear_vel == 0. and current_vel < 0:
            throttle = 0
            brake = MAMAX_TORQUE  # N.m, to hold car in place, acceleration ~ 1 m/s2 -> 1736 kg * 1 m/s2 * 0.2413 m = 418.9 N.m

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.vehicle_params.decel_limit)
            brake = abs(decel) * self.vehicle_params.total_vehicle_mass * self.vehicle_params.wheel_radius  # Torque N.m

        # rospy.logwarn("Target velocity: {0}".format(linear_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))


        return throttle, brake, steering

