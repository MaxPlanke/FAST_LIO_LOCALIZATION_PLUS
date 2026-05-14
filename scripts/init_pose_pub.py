#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from fast_lio.msg import RelocalizationMsg
from tf.transformations import quaternion_from_euler

rospy.init_node('localization_init_pose_pub')
pub = rospy.Publisher('/relocalization', RelocalizationMsg, queue_size=1, latch=True)

# Create RelocalizationMsg
relocalization_msg = RelocalizationMsg()

# Set PCD path
relocalization_msg.pcd_path = "/home/amov/ws_FASTLIO_KFY/helmet_pdr_S1_stair_2026_05_13_B4_B3/12834 - Block S1_voxel_0p10.pcd"

# Set initial pose
relocalization_msg.init_pose = PoseStamped()
relocalization_msg.init_pose.header.frame_id = "map"
relocalization_msg.init_pose.header.stamp = rospy.Time.now()

relocalization_msg.init_pose.pose.position.x = 91.226
relocalization_msg.init_pose.pose.position.y = -9.672
relocalization_msg.init_pose.pose.position.z = -3.316

roll = 0.0  # 6 degrees in radians
pitch = 0.3840  # 22 degrees in radians
yaw = 1.5708  # 90 degrees in radians

quaternion = quaternion_from_euler(roll, pitch, yaw)
relocalization_msg.init_pose.pose.orientation.x = quaternion[0]
relocalization_msg.init_pose.pose.orientation.y = quaternion[1]
relocalization_msg.init_pose.pose.orientation.z = quaternion[2]
relocalization_msg.init_pose.pose.orientation.w = quaternion[3]

pub.publish(relocalization_msg)
rospy.sleep(1.0)
