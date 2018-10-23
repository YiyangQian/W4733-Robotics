import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

pub = rospy.Publisher('state', PointStamped, queue_size=10)
markerPub = rospy.Publisher('vgraph_markers', Marker, queue_size=10)

rospy.init_node('stateNode', anonymous=True)

rate = rospy.Rate(1)

state = PointStamped()

# initial starting location I might want to move to the param list
h = rospy.get_param("height", 0)
w = rospy.get_param("width", 0)
state.point.x = h
state.point.y = w
state.point.z = 10

robotMarker = Marker()
robotMarker.header.frame_id = "/Cmap"
robotMarker.header.stamp    = rospy.get_rostime()
robotMarker.ns = "robot"
robotMarker.id = 0
robotMarker.type = 2 # sphere
robotMarker.action = 0
robotMarker.pose.position.x = 1
robotMarker.pose.position.y = 1
robotMarker.pose.position.z = 1

robotMarker.pose.orientation.x = 0
robotMarker.pose.orientation.y = 0
robotMarker.pose.orientation.z = 0
robotMarker.pose.orientation.w = 1.0
robotMarker.scale.x = 1.0
robotMarker.scale.y = 0.1
robotMarker.scale.z = 0.1

robotMarker.color.r = 0.0
robotMarker.color.g = 1.0
robotMarker.color.b = 0.0
robotMarker.color.a = 1.0

robotMarker.lifetime = rospy.Duration(0)

while not rospy.is_shutdown():
    markerPub.publish(robotMarker)
    print('haha')
    rate.sleep()



