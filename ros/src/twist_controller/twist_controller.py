import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        kp = 5.0
        ki = 0.05
        kd = 0.01
        mn = decel_limit
        mx = accel_limit
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        tau = 0.2
        ts = 0.1
        self.vel_lpf = LowPassFilter(tau, ts)
        self.brake_lpf = LowPassFilter(0.5, 0.02)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.brake_max = 0.6
        self.max_brake_const = self.brake_max * self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius

        self.last_time = None
        self.applied_brake = 0

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.0
        brake = 0.0
        steering = 0.0

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.last_time = rospy.get_time()
            return 0., 0., 0.

        if self.last_time is None:
            self.last_time = rospy.get_time()
            return throttle, brake, steering

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        throttle = self.throttle_controller.step(vel_error, sample_time)
        # throttle = self.vel_lpf.filt(throttle)

        # Here to set the velocity threshold so that, as the speed is too low, just stop
        # In order to avoid the jerk

        if throttle >= 0.0:
            throttle = 0.75 * math.tanh(throttle * 0.6)
            if throttle > self.accel_limit:
                throttle = self.accel_limit
            if throttle < self.decel_limit:
                throttle = self.decel_limit
            brake = 0
        elif throttle < -1.1:
            brake = self.max_brake_const * math.tanh(-throttle * 0.3)
            throttle = 0
        else:
            brake = 0
            throttle = 0

        brake = self.brake_lpf.filt(brake)
        if brake < 100:
            brake = 0.0
        # decel = max(vel_error, self.decel_limit)
        # brake = decel * self.vehicle_mass * self.wheel_radius if decel > 0 else 0

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel,
                                                    current_vel)  # if vel_error == 0 and throttle == 0:
        # 	brake = 0.0

        return throttle, brake, steering
