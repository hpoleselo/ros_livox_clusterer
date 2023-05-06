"""
livox_ros_driver/CustomMsg (from Mid-70) to PointCloud2 converter node.
The goal of this package is to make the point cloud available to be visualized in RViz.

Author: Henrique Poleselo
May 6th 2023.
"""
#!/usr/bin/env python3

import rospy
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import struct

# TODO: To be set in the the launch file
TOPIC_NAME = '/livox/lidar'

# TODO: To be set in the launch file
OUTPUT_TOPIC_NAME = '/livox/lidar/converted'

pc2_publisher = rospy.Publisher(OUTPUT_TOPIC_NAME, PointCloud2, queue_size=10)

def callback(livox_custom_msg: CustomMsg):
    pc2_msg = custom_msg_to_pc2(livox_custom_msg)
    pc2_publisher.publish(pc2_msg)

def custom_msg_to_pc2(livox_custom_msg: CustomMsg):
    """
    Converts custom message to PointCloud2 and published to a new topic.
    """

    # Header fields
    """
    pc2_msg = PointCloud2()
    pc2_msg.header.seq = livox_custom_msg.header.seq
    pc2_msg.header.stamp.secs = livox_custom_msg.header.stamp
    pc2_msg.header.frame_id = livox_custom_msg.header.frame_id
    """

    # Instead using a Python function which does all that for us
    header = Header()
    header.seq = livox_custom_msg.header.seq
    # TODO: needs to be properbly serialized before sent
    #header.stamp.secs = livox_custom_msg.header.stamp
    header.frame_id = livox_custom_msg.header.frame_id

    # Point Cloud Data: PC2 takes in PointField for each dimension (x,y,z)
    fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgb', 12, PointField.UINT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),
    ]
    
    points = []
    x = livox_custom_msg.points[0].x
    y = livox_custom_msg.points[0].y
    z = livox_custom_msg.points[0].z
    #r = int(x * 255.0)
    #g = int(y * 255.0)
    #b = int(z * 255.0)
    #a = 255
    #rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
    pt = [x, y, z, 0]
    points.append(pt)
    
    # create_cloud.py deals with the serialization of Point Cloud 2 message
    pc2_msg = point_cloud2.create_cloud(header, fields, points)

    return pc2_msg
    

def listener():
    rospy.init_node('livox_to_pc2_converter', anonymous=True)

    rospy.Subscriber(TOPIC_NAME, CustomMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()