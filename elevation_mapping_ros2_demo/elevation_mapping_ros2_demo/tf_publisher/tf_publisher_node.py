import sys
import numpy as np

import rclpy
import rclpy.time
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs
from tf2_ros import TypeException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

class TfPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("sensor_frame", "zedm_base_link")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.is_first_tf = False
        self.publish_tf()
        
        self.publisher = self.create_publisher(geometry_msgs.PoseWithCovarianceStamped, 'pose_covariance', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        
    def publish_tf(self):
        if not self.is_first_tf:
            self.is_first_tf = True
            self.time_first_tf: rclpy.time.Time = self.get_clock().now()
            
        # publish transform from map to robot
        duration: rclpy.time.Duration = self.get_clock().now() - self.time_first_tf
        map_to_robot = geometry_msgs.TransformStamped()
        map_to_robot.header.stamp = self.get_clock().now().to_msg()
        map_to_robot.header.frame_id = self.get_parameter("map_frame").get_parameter_value().string_value
        map_to_robot.child_frame_id = self.get_parameter("robot_frame").get_parameter_value().string_value
        map_to_robot.transform.translation.x = 2.0
        map_to_robot.transform.translation.y = 6.0# + 0.2*float(duration.nanoseconds) / 1e9
        map_to_robot.transform.translation.z = 0.0
        map_to_robot.transform.rotation.x = 0.0
        map_to_robot.transform.rotation.y = 0.0
        map_to_robot.transform.rotation.z = 0.0
        map_to_robot.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(map_to_robot)
        
        
        # publish transform from robot to sensor
        robot_to_sensor = geometry_msgs.TransformStamped()
        robot_to_sensor.header.stamp = self.get_clock().now().to_msg()
        robot_to_sensor.header.frame_id = self.get_parameter("robot_frame").get_parameter_value().string_value
        robot_to_sensor.child_frame_id = self.get_parameter("sensor_frame").get_parameter_value().string_value
        robot_to_sensor.transform.translation.x = 2.0
        robot_to_sensor.transform.translation.y = 6.0
        robot_to_sensor.transform.translation.z = 0.0
        robot_to_sensor.transform.rotation.x = 0.0
        robot_to_sensor.transform.rotation.y = 0.0
        robot_to_sensor.transform.rotation.z = 0.0
        robot_to_sensor.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(robot_to_sensor)
         
        
    def timer_callback(self):
        self.publish_tf()
        
        # publish pose with covariance stamped msg
        target_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        origin_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        try:
            transform: geometry_msgs.TransformStamped = self.tf_buffer.lookup_transform(
                origin_frame, target_frame, rclpy.time.Time()
            )
        except TypeException as ex:
            self.get_logger().info(
                f'Cloud not transform {origin_frame} to {target_frame}: {ex}'
            )
            return
        
        # self.get_logger().info(f"Get transform from {origin_frame} to {target_frame}:\
        #     x:{transform.transform.translation.x} \
        #     y:{transform.transform.translation.y} \
        #     z:{transform.transform.translation.z} \
        #     qx:{transform.transform.rotation.x} \
        #     qy:{transform.transform.rotation.y} \
        #     qz:{transform.transform.rotation.z} \
        #     qw:{transform.transform.rotation.w}")
        
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
        
def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    rclpy.spin(tf_publisher)
    
    tf_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()