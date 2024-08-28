import sys
import numpy as np
import time

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs
from tf2_ros import TypeException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

class TfPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.publish_tf()
        
        self.publisher = self.create_publisher(geometry_msgs.PoseWithCovarianceStamped, 'pose_with_covariance', 10)
        
        time.sleep(0.5)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def publish_tf(self):
        stamp = self.get_clock().now()
        # publish transform from map to robot
        pcd2robot = geometry_msgs.TransformStamped()
        pcd2robot.header.stamp = stamp.to_msg()
        pcd2robot.header.frame_id = "pcd_link"
        pcd2robot.child_frame_id = "base_link"
        pcd2robot.transform.translation.x = 2.0
        pcd2robot.transform.translation.y = 5.0# + 0.2*float(duration.nanoseconds) / 1e9
        pcd2robot.transform.translation.z = 0.0
        pcd2robot.transform.rotation.x = 0.0
        pcd2robot.transform.rotation.y = 0.0
        pcd2robot.transform.rotation.z = 0.0
        pcd2robot.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(pcd2robot)
        
        
        # publish transform from robot to sensor
        robot2perfect = geometry_msgs.TransformStamped()
        robot2perfect.header.stamp = stamp.to_msg()
        robot2perfect.header.frame_id = "base_link"
        robot2perfect.child_frame_id = "perfect_camera_link"
        robot2perfect.transform.translation.x = 0.2
        robot2perfect.transform.translation.y = 0.0
        robot2perfect.transform.translation.z = 0.1
        robot2perfect.transform.rotation.x = 0.0
        robot2perfect.transform.rotation.y = 0.0
        robot2perfect.transform.rotation.z = 0.0
        robot2perfect.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(robot2perfect)

        robot2stereo = geometry_msgs.TransformStamped()
        robot2stereo.header.stamp = stamp.to_msg()
        robot2stereo.header.frame_id = "base_link"
        robot2stereo.child_frame_id = "stereo_camera_link"
        robot2stereo.transform.translation.x = 0.2
        robot2stereo.transform.translation.y = 0.0
        robot2stereo.transform.translation.z = 0.2
        robot2stereo.transform.rotation.x = 0.0
        robot2stereo.transform.rotation.y = 0.0
        robot2stereo.transform.rotation.z = 0.0
        robot2stereo.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(robot2stereo)


        robot2laser = geometry_msgs.TransformStamped()
        robot2laser.header.stamp = stamp.to_msg()
        robot2laser.header.frame_id = "base_link"
        robot2laser.child_frame_id = "laser_link"
        robot2laser.transform.translation.x = 0.2
        robot2laser.transform.translation.y = 0.0
        robot2laser.transform.translation.z = 2.0
        robot2laser.transform.rotation.x = 0.0
        robot2laser.transform.rotation.y = np.sin(np.pi/4.0)
        robot2laser.transform.rotation.z = 0.0
        robot2laser.transform.rotation.w = np.cos(np.pi/4.0)
        self.tf_broadcaster.sendTransform(robot2laser)

        
        map2pcd = geometry_msgs.TransformStamped()
        map2pcd.header.stamp = stamp.to_msg()
        map2pcd.header.frame_id = "map"
        map2pcd.child_frame_id = "pcd_link"
        map2pcd.transform.translation.x = 0.0
        map2pcd.transform.translation.y = 0.0
        map2pcd.transform.translation.z = 3.0
        map2pcd.transform.rotation.x = 0.0
        map2pcd.transform.rotation.y = 0.0
        map2pcd.transform.rotation.z = 0.0
        map2pcd.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(map2pcd)
        
        self.last_publish_time = stamp
         
        
    def timer_callback(self):        
        # publish pose with covariance stamped msg
        target_frame = "base_link"
        origin_frame = "map"
        try:
            transform: geometry_msgs.TransformStamped = self.tf_buffer.lookup_transform(
                origin_frame, target_frame, self.last_publish_time
            )
            success = True
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().info(
                f'Cloud not transform {origin_frame} to {target_frame}: {ex}'
            )
            success = False
        
        if success:
            msg = geometry_msgs.PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = origin_frame
            msg.pose.pose.position.x = transform.transform.translation.x
            msg.pose.pose.position.y = transform.transform.translation.y
            msg.pose.pose.position.z = transform.transform.translation.z
            msg.pose.pose.orientation.x = transform.transform.rotation.x
            msg.pose.pose.orientation.y = transform.transform.rotation.y
            msg.pose.pose.orientation.z = transform.transform.rotation.z
            msg.pose.pose.orientation.w = transform.transform.rotation.w
            
            msg.pose.covariance = np.zeros(36)
            
            self.publisher.publish(msg)
        self.publish_tf()
        
        
def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    rclpy.spin(tf_publisher)
    
    tf_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()