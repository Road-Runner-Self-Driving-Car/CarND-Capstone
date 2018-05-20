
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        kp = 0.9
        ki = 0.0005
        kd = 0.07
        mn = decel_limit
        mx = accel_limit
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
		self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        tau = 0.2
        ts = 0.1
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.applied_brake = 0

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

       	if not dbw_enabled:
       		self.throttle_controller.reset()
       		return 0., 0., 0.,
       	# current_vel = self.vel_lpf.filt(current_vel)
    	steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

    	vel_error = linear_vel - current_vel
    	self.last_vel = current_vel

    	current_time = rospy.get_time()
    	sample_time = current_time - self.last_time
    	self.last_time  = current_time

	    throttle = self.throttle_controller.step(vel_error, sample_time)
		throttle = self.vel_lpf.filt(throttle)
    	
		brake = 0.0
    	# Here to set the velocity threshold so that, as the speed is too low, just stop
    	# In order to avoid the jerk
    	if abs(linear_vel)  < 0.01  and abs(current_vel) < 0.3:
    		throttle = self.decel_limit

    	if throttle < 0:
			decel = abs(throttle)
    		throttle = 0
    		# decel = max(vel_error, self.decel_limit)
    		brake = decel *self.vehicle_mass*self.wheel_radius if decel > 0 else 0

    	# if vel_error == 0 and throttle == 0:
    	# 	brake = 0.0

    	return throttle, brake, steering	
