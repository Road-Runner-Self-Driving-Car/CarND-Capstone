#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

print("passed")

from scipy.spatial import KDTree
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number



# global states
ST_INITIAL = 0
ST_GO = 1


class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)    

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # NOTE: comment out when traffic_waypoint done...
        # rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_cb)               
        # NOTE: uncomment when traffic lights working...
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = Waypoint()

        self.state = ST_INITIAL
        self.restricted_speed_mps = 0.0 # mps set it to zero at the beginnig until we get the /base_waypoints
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.cruise_control = None
        self.cruise_c_poly = None
        self.decel_poly = None
        self.lights = []       
        self.decels = None
        self.cwp = None
        self.redtlwp = None
        self.i = 0
        self.go_timer = 0
        self.theta = None  # from euler


        self.loop()
        #rospy.spin()

    def loop(self):
        # update rate / sec
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.state != ST_INITIAL and self.pose and self.base_waypoints:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_cords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
                
    def pose_cb(self, msg):
        # TODO: Implement
        self.i += 1
        self.pose = msg.pose
        self.position = self.pose.position
        self.orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w])
        self.theta = euler[2]
        if self.state == ST_INITIAL:
            print "Traffic Light Detector Initialized ..."

           

    def waypoints_cb(self, waypoints):
        # TODO: Implement

       MPS = 0.44704

#prev:
        self.base_waypoints = waypoints
        #if not self.waypoints_2d:
        #    self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        #    self.waypoint_tree = KDTree(self.waypoints_2d)


        # the waypoints are static - make own copy
        global LOOKAHEAD_WPS
        if self.waypoints_2d is None:

            r = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

            # unsubscribe to the waypoint messages - cut down on resource usage
            self.sub_waypoints.unregister()
            self.sub_waypoints = None

            # set the restricted speed limit
            self.restricted_speed_mps = msg.waypoints[0].twist.twist.linear.x

            # create our own copy of the waypoint array
            self.waypoints_2d = []
            vptsd = []
           
            p0 = msg.waypoints[0].pose.pose.position
            wpd = 0.
            for waypoint in msg.waypoints:
            
                # calculate distances
                p1 = waypoint.pose.pose.position
                ld = r(p0, p1)
                vptsd.append(wpd+ld)
                p0 = p1
                wpd += ld

                # create own copy of waypoint array
                p = Waypoint()
                p.pose.pose.position.x = waypoint.pose.pose.position.x
                p.pose.pose.position.y = waypoint.pose.pose.position.y
                p.pose.pose.position.z = waypoint.pose.pose.position.z
                p.pose.pose.orientation.x = waypoint.pose.pose.orientation.x
                p.pose.pose.orientation.y = waypoint.pose.pose.orientation.y
                p.pose.pose.orientation.z = waypoint.pose.pose.orientation.z
                p.pose.pose.orientation.w = waypoint.pose.pose.orientation.w
                p.twist.twist.linear.x = waypoint.twist.twist.linear.x
                p.twist.twist.linear.y = waypoint.twist.twist.linear.y
                p.twist.twist.linear.z = waypoint.twist.twist.linear.z
                p.twist.twist.angular.x = waypoint.twist.twist.angular.x
                p.twist.twist.angular.y = waypoint.twist.twist.angular.y
                p.twist.twist.angular.z = waypoint.twist.twist.angular.z
                self.waypoints_2d.append(p)

            if LOOKAHEAD_WPS > len(msg.waypoints):
                LOOKAHEAD_WPS = len(msg.waypoints)

            # create our deceleration points (in reverse) from restricted speed limit 
            wpx0 = [-LOOKAHEAD_WPS, 0., LOOKAHEAD_WPS]
            wpy0 = [self.restricted_speed_mps, self.restricted_speed_mps, self.restricted_speed_mps]
            poly0 = np.polyfit(np.array(wpx0), np.array(wpy0), 2)
            self.cruise_c_poly = np.poly1d(poly0)

            wpx1 = []
            wpy1 = []

            wpx1.append(-LOOKAHEAD_WPS)
            wpy1.append(-0.1)

            wpx1.append(0.)
            wpy1.append(-0.2)

            # 5 metres away
            wpx1.append(5)
            wpy1.append(MPS * 0.5)
            wpx1.append(10)
            wpy1.append(MPS * 5)
            wpx1.append(16)
            wpy1.append(MPS * 5)

            # 2 seconds away
            wpx1.append(max([self.restricted_speed_mps * 2, 24]))
            wpy1.append(max([self.restricted_speed_mps * 0.2, MPS * 5]))

            # 4 seconds away
            wpx1.append(max([self.restricted_speed_mps * 4, 45]))
            wpy1.append(max([self.restricted_speed_mps * 0.3, MPS * 6]))

            # 6 seconds away
            wpx1.append(max([self.restricted_speed_mps * 6, 65]))
            wpy1.append(max([self.restricted_speed_mps * 0.5, MPS * 10]))

            # 8 seconds away
            wpx1.append(max([self.restricted_speed_in_mps * 8, 85]))
	        wpy1.append(self.restricted_speed_mps)

            wpx1.append(LOOKAHEAD_WPS)
            wpy1.append(self.restricted_speed_mps)

            poly1 = np.polyfit(np.array(wpx1), np.array(wpy1), 3)
            self.decel_poly = np.poly1d(poly1)

            # use the -0.01 speed to create our stop points
            wpx2 = [-LOOKAHEAD_WPS, 0., LOOKAHEAD_WPS]
            wpy2 = [-0.01, -0.01, -0.01]
            poly2 = np.polyfit(np.array(wpx2), np.array(wpy2), 2)
            self.stop_poly = np.poly1d(poly2)

            wlen = len(self.waypoints_2d)
            velpolypoint = vptsd[wlen-1]
            for i in range(LOOKAHEAD_WPS):
                ideal_velocity = self.decelpoly([(velpolypoint - vptsd[wlen-1 - LOOKAHEAD_WPS + i])])[0]
                self.waypoints_2d[wlen-1-LOOKAHEAD_WPS + i].twist.twist.linear.x = ideal_velocity
        
        self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.redtlwp = msg.data
        if self.state == ST_INITIAL and self.i > 200:
            self.state = ST_GO # no longer in startup state       

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

