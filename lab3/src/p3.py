import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from math import radians, pi

from convex_hull import *


class P3():
    def __init__(self):
        rospy.init_node('p3', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        quaternions = list()
        
        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        convex_hull_array = calculate_convex_hull([[-18,-18],[-18,18],[18,-18],[18,18]])
        for i in range(len(convex_hull_array)):
            this_convex_waypoints = list()
            current_convex_hull = convex_hull_array[i]
            print(current_convex_hull)
            for j in range(current_convex_hull.shape[0]):
                new_x = current_convex_hull[j][0]/100
                new_y = current_convex_hull[j][1]/100
                this_convex_waypoints.append(Pose(Point( new_x, new_y , 0.0), quaternions[0]))                    
            waypoints.append(this_convex_waypoints)

        self.edges = set()
        for i in range(len(convex_hull_array)):
            current_convex_hull = convex_hull_array[i]
            for j in range(current_convex_hull.shape[0]):
                cur_x = current_convex_hull[j][0]/100
                cur_y = current_convex_hull[j][1]/100
                next_index = (j + 1) % current_convex_hull.shape[0]
                next_x = current_convex_hull[next_index][0]/100
                next_y = current_convex_hull[next_index][1]/100
                self.edges.add(((cur_x,cur_y),(next_x, next_y)))
        print(len(self.edges))
        
        # Initialize the visualization markers for RViz

        self.init_markers(len(convex_hull_array))
        # Set a visualization marker at each waypoint        
        
            
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        

        for i in range(len(convex_hull_array)):
            #self.markers.points = list()
            for waypoint in waypoints[i]:           
                p = Point()
                p = waypoint.position
                self.marker_array.markers[i].points.append(p)

            self.marker_array.markers[i].points.append(waypoints[i][0].position)

            self.marker_pub.publish(self.marker_array.markers[i])
            rospy.sleep(0.5)

        for i in range(len(convex_hull_array)):
            current_convex_hull = convex_hull_array[i]
            for j in range(len(current_convex_hull)):
                p1 = current_convex_hull[j]
                for m in range(len(convex_hull_array)):
                    if (m != i):
                        for n in range(len(convex_hull_array[m])):
                            p2 = convex_hull_array[m][n]
                            if self.isLineExisting(p1, p2):
                                # print(p1,p2, "I am here ", self.marker_id)
                                markers = Marker()
                                marker_scale = 0.03
                                marker_lifetime = 0 # 0 is forever
                                marker_ns = 'vgraph'
                                marker_color = {'r': 1.0, 'g': 0, 'b': 0, 'a': 1.0}
                                markers.ns = 'vgraph'
                                markers.id = self.marker_id
                                markers.type = Marker.LINE_STRIP
                                markers.action = Marker.ADD
                                markers.lifetime = rospy.Duration(marker_lifetime)
                                markers.scale.x = marker_scale
                                markers.scale.y = marker_scale
                                markers.color.r = marker_color['r']
                                markers.color.g = marker_color['g']
                                markers.color.b = marker_color['b']
                                markers.color.a = marker_color['a']
                                
                                markers.header.frame_id = 'odom'
                                markers.header.stamp = rospy.Time.now()
                                markers.points = list()
                                markers.points.append(Point(p1[0]/100,p1[1]/100,0))
                                markers.points.append(Point(p2[0]/100,p2[1]/100,0))
                                
                                self.marker_array.markers.append(markers)  
                                self.marker_pub.publish(self.marker_array.markers[-1])
                                self.marker_id +=1
                                rospy.sleep(0.5)


    def init_markers(self, convex_size):
        # Set up our waypoint markers
        # Define a marker publisher.
        self.marker_array = MarkerArray()
        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=5)

        marker_scale = 0.03
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'vgraph'
        self.marker_id = 0
        marker_color = {'r': 1.0, 'g': 0, 'b': 0, 'a': 1.0}
        

        for i in range(convex_size):
            # Initialize the marker points list.
            markers = Marker()
            markers.ns = marker_ns
            markers.id = self.marker_id
            markers.type = Marker.LINE_STRIP
            markers.action = Marker.ADD
            markers.lifetime = rospy.Duration(marker_lifetime)
            markers.scale.x = marker_scale
            markers.scale.y = marker_scale
            markers.color.r = marker_color['r']
            markers.color.g = marker_color['g']
            markers.color.b = marker_color['b']
            markers.color.a = marker_color['a']
            
            markers.header.frame_id = 'odom'
            markers.header.stamp = rospy.Time.now()
            markers.points = list()
            self.marker_id +=1
            self.marker_array.markers.append(markers)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def isLineExisting(self, p1, p2):

        p1 = (p1[0]/100, p1[1]/100)
        p2 = (p2[0]/100, p2[1]/100)
        for edge in self.edges:
            q1, q2 = edge
            if self.isIntersected(p1, p2, q1, q2):
                return False
        return True
    
    def isIntersected(self, p1, p2, p3, p4):
        print("p1 ", p1, "p2 ", p2, "p3 ", p3, "p4 ", p4)
        res = False
        if (max(p1[0], p2[0]) > min(p3[0], p4[0])
        and max(p3[0], p4[0]) > min(p1[0], p2[0])
        and max(p1[1], p2[1]) > min(p3[1], p3[1])
        and max(p3[1], p4[1]) > min(p1[1], p2[1])):
            if (self.cross(p1, p2, p3) * self.cross(p1, p2, p4) < 0 and self.cross(p3, p4, p1) * self.cross(p3, p4, p2) < 0):
                res = True
            else:
                res = False
        else:
            res = False
        # print(res)
        return res

    def cross(self, p1, p2, p3):
        x1 = p2[0] - p1[0]
        y1 = p2[1] - p1[1]
        x2 = p3[0] - p1[0]
        y2 = p3[1] - p1[1]
        return x1*y2-x2*y1

if __name__ == '__main__':
    try:
        P3()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
