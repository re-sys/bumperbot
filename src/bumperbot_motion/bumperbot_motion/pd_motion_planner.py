#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix,translation_from_matrix, inverse_matrix

class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")
        self.declare_parameter("kp",5.0)
        self.declare_parameter("kd",0.1)
        self.declare_parameter("step_size",0.2)
        self.declare_parameter("max_linear_velocity",5.0)
        self.declare_parameter("max_angular_velocity",3.0)

        self.kp = self.get_parameter("kp").value
        self.kd = self.get_parameter("kd").value
        self.step_size = self.get_parameter("step_size").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        self.path_sub = self.create_subscription(Path, "/a_star/path",self.path_callback,10)
        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.next_pose_pub = self.create_publisher(PoseStamped,"/pd/next_pose",10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1,self.control_loop)

        self.global_plan = None
        self.pre_linear_error = 0.0
        self.pre_angular_error = 0.0
        self.last_time_cycle = self.get_clock().now()

    def path_callback(self,path : Path):
        self.global_plan = path
        self.get_logger().info("get plan!!!!!!!!!!!")

    def control_loop(self):
        if not self.global_plan or not self.global_plan.poses:
            self.get_logger().info(f"no global_plan receive"
                                   f"hello world"
                                   ,throttle_duration_sec=0.2)
            return
        
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform("odom","base_footprint",rclpy.time.Time())
        except Exception as ex:
            self.get_logger().warn(f"Could not transform:{ex}")
            return
        
        # self.get_logger().info(f"frame_id Robot Pose: {robot_pose_transform.header.frame_id}")
        # self.get_logger().info(f"frame_id Global Plan {self.global_plan.header.frame_id}")
        if not self.transform_frame(robot_pose_transform.header.frame_id):
            self.get_logger().warn(f"ubable to transform plan to robots frame")
            return 

        robot_pose = PoseStamped()
        robot_pose.header.frame_id =  robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation
        next_pose: PoseStamped = self.get_next_pose(robot_pose)

        dx = next_pose.pose.position.x - robot_pose.pose.position.x
        dy = next_pose.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx*dx+dy*dy)
        if distance <=0.1:
            self.get_logger().info("Goal Received")
            return
        self.next_pose_pub.publish(next_pose)
        robot_tf = self.get_tf_from_pose(robot_pose)
        next_pose_tf = self.get_tf_from_pose(next_pose)
        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf),next_pose_tf)

        linear_error = next_pose_robot_tf[0][3]
        angular_error = next_pose_robot_tf[1][3]

        dt = (self.get_clock().now()-self.last_time_cycle).nanoseconds*1e-9
        linear_error_derivative = (linear_error - self.pre_linear_error)/dt
        angular_error_derivative = (angular_error - self.pre_angular_error)/dt

        self.last_time_cycle = self.get_clock().now()
        self.pre_angular_error = angular_error
        self.pre_linear_error = linear_error

        cmd_vel = Twist()
        cmd_vel.linear.x = max(-self.max_linear_velocity,
                             min(self.kp*linear_error+self.kd*linear_error_derivative,self.max_linear_velocity))
        cmd_vel.angular.z = max(-self.max_angular_velocity,
                             min(self.kp*angular_error+self.kd*angular_error_derivative,self.max_angular_velocity))
        self.cmd_pub.publish(cmd_vel)

        

    def get_tf_from_pose(self,pose:PoseStamped):
        tf = quaternion_matrix([pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z,
                                pose.pose.orientation.w])
        tf[0][3] = pose.pose.position.x
        tf[1][3] = pose.pose.position.y
        return tf

    def transform_frame(self,frame):
        plan_frame = self.global_plan.header.frame_id
        if plan_frame == frame:
            return True

        try:
            transform = self.tf_buffer.lookup_transform(frame,plan_frame,rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error(f"Could not find transform from {plan_frame} to {frame}")
            return False
        
        rotation = transform.transform.rotation
        transform_matrix = quaternion_matrix([rotation.x,
                                              rotation.y,
                                              rotation.z,
                                              rotation.w])
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y

        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix([pose.pose.orientation.x,
                                             pose.pose.orientation.y,
                                             pose.pose.orientation.z,
                                             pose.pose.orientation.w,])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y
            transformed_pose = concatenate_matrices(transform_matrix,pose_matrix)
            [pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x,pose.pose.position.y,_] = translation_from_matrix(transformed_pose)
            pose.header.frame_id = frame

        self.global_plan.header.frame_id = frame
        return True

    def get_next_pose(self, robot_pose:PoseStamped):
        next_pose = self.global_plan.poses[-1]
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx*dx+dy*dy)
            if distance >self.step_size:
                next_pose = pose
            else:
                break
        return next_pose
            
            

            
        



def main():
    rclpy.init()
    pd_motion_node = PDMotionPlanner()
    rclpy.spin(pd_motion_node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
