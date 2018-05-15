#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import numpy as np
from scipy.spatial import KDTree

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

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_waypoints = None
        self.pose = None        
        self.stopline_wp_idx = -1
        self.kdtree_waypoints = None


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.waypoints_2d = None
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
      
        self.traffic_cb = None
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size = 1)

        self.loop()


    def loop(self):
        rate = rospy.Rate(50)   # update rate / sec
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:           
                self.publish_waypoints()
            rate.sleep()


    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.kdtree_waypoints.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        # Dot product
        v = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if v > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def deceleration_waypoints(self, waypoints, closest_idx):
        MAX_DECELERATION = 0.5
        
        new_points = []

        for i, wp in enumerate(waypoints):
            pnt = Waypoint()
            pnt.pose = wp.pose

            # Stop 2 or 3 waypoints back so nose of car stops on line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # start with 2
            dist = self.distance(waypoints, i, stop_idx)
            
            velocity = math.sqrt( 2 * MAX_DECELERATION * dist)   # note that linear may be better than sqrt
            if velocity < 1.0:
                velocity = 0.0
            pnt.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            new_points.append(pnt)
        
        return new_points


     
    def publish_waypoints(self):
        
        # Build final lane
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_wpoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_wpoints
        else:
            lane.waypoints = self.deceleration_waypoints(base_waypoints, closest_idx)

        # publish        
        self.final_waypoints_pub.publish(lane)   



    def pose_cb(self, msg):
        # DONE: Implement

        # TODO: can do more here 
        self.pose = msg  
        
        #self.i += 1
        #self.pose = msg.pose
        #self.position = self.pose.position
        #self.orientation = self.pose.orientation
        #euler = tf.transformations.euler_from_quaternion([
        #    self.orientation.x,
        #    self.orientation.y,
        #    self.orientation.z,
        #    self.orientation.w])
        #self.theta = euler[2]
        #if self.state == ST_INITIAL:
        #    print "Traffic Light Detector Initialized ..."



    def waypoints_cb(self, waypoints):
        # DONE: Implement        
        # Load the base waypoints once
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            # convert the data structure to KDTree So that it is fast to calculate closest waypoint
            self.waypoints_2d = [[wpt.pose.pose.position.x, wpt.pose.pose.position.y] for wpt in waypoints.waypoints]
            self.kdtree_waypoints = KDTree(self.waypoints_2d)
        

    def traffic_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        pass

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
