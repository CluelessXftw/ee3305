from math import hypot, atan2, inf, cos, sin, isinf, isnan, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from math import atan2, sin, cos, pi



class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").get_parameter_value().double_value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").get_parameter_value().double_value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").get_parameter_value().double_value
        self.stop_thres_ = self.get_parameter("stop_thres").get_parameter_value().double_value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").get_parameter_value().double_value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").get_parameter_value().double_value

        # Handles: Topic Subscribers
        # Subscribers
        self.sub_path_ = self.create_subscription(
            Path, 'path', self.callbackSubPath_, 10
        )
        self.sub_odom_ = self.create_subscription(
            Odometry, 'odom', self.callbackSubOdom_, 10
        )
        self.sub_laserscan_ = self.create_subscription(
            LaserScan, 'scan', self.callbackSubLaserScan_, qos_profile_sensor_data
        )

        # Publishers
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, 'cmd_vel', 10
        )
        self.pub_lookahead_ = self.create_publisher(
            PoseStamped, 'lookahead_point', 10
        )
        

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Other Instance Variables
        self.path_poses = []
        self.received_odom_ = False
        self.received_path_ = False
        self.obstacle_slowdown_threshold_ = 0.5  # robot slows down here
        self.obstacle_stop_threshold_ = 0.1      # robot stops here

    # Callbacks =============================================================
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:  # not msg.poses is fine but not clear
            self.get_logger().warn(f"Received path message is empty!")
            return  # do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.

        # !TODO: copy the array from the path
        self.path_poses_ = list(msg.poses)

        self.received_path_ = True
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")


    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        # !TODO: write robot pose to rbt_x_, rbt_y_, rbt_yaw_
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # Yaw from quaternion using atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))
        self.rbt_yaw_ = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.received_odom_ = True
      

    # LaserScan subscriber callback
    def callbackSubLaserScan_(self, msg: LaserScan):
        self.latest_laserscan = msg  # store most recent scan for use in timer


    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot
        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        speed = self.lookahead_lin_vel_ 

        # From the closest point, iterate towards the goal and find the first point that is at least a lookahead distance away.
        # Return the goal point if no more lookahead point
        lookahead_idx = len(self.path_poses_) - 1
        for i in range(closest_idx, len(self.path_poses_)):
            px = self.path_poses_[i].pose.position.x
            py = self.path_poses_[i].pose.position.y
            dx = px - self.rbt_x_
            dy = py - self.rbt_y_
            dist = hypot(dx, dy)
            adaptive_lookahead_distance = self.getAdaptiveLookaheadDistance(speed, dx, dy)
            if dist >= adaptive_lookahead_distance:
                lookahead_idx = i
                break   # Stop at first point that satisfies lookahead distance

        # Get the lookahead coordinates
        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Return the coordinates
        return lookahead_x, lookahead_y
    
    def getAdaptiveLookaheadDistance(self, speed, dx, dy): # IMPROVEMENT: Adaptive LookAhead Distance
        # proportional to current max linear velocity or speed
        # ensure minimum and maximum bounds for lookahead distance
        speed = self.lookahead_lin_vel_

        # Heading error
        heading_to_goal = atan2(dy, dx)
        heading_error = heading_to_goal - self.rbt_yaw_
        # Normalize heading_error to [-pi, pi]
        heading_error = (heading_error + pi) % (2 * pi) - pi

        # Reduce lookahead for sharp turns (large heading error)
        heading_factor = max(0.3, 1.0 - abs(heading_error) / pi)  # 0.3~1.0

        lookahead = max(0.2, min(1.0, speed * 2.0 * heading_factor))
        return lookahead

    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        if not self.received_odom_ or not self.received_path_:
            return  # return silently if path or odom is not received.

        # get lookahead point
        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        # get distance to lookahead point (not to be confused with lookahead_distance)
        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        dist_to_lookahead = hypot(dx, dy)
        

        def shortest_angle_diff(a, b):
            # return signed smallest angle from b --> a, in [-pi, pi]
            d = a - b
            return atan2(sin(d), cos(d))

        angle_to_point = atan2(dy, dx)
        angle_diff = shortest_angle_diff(angle_to_point, self.rbt_yaw_)

        # If target is behind (use 90°) OR if angle error large enough 
        if abs(angle_diff) > (pi / 2.0) or abs(angle_diff) > 0.6:  
            lin_vel = 0.0
            ang_vel = self.max_ang_vel_ * (1.0 if angle_diff > 0 else -1.0)

            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = lin_vel
            msg_cmd_vel.twist.angular.z = ang_vel
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            
            self.get_logger().info("Spin Modeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
            
            return

# Process LaserScan to find the minimum distance to an obstacle
        if not hasattr(self, "latest_laserscan"):
            return
        
        valid_ranges = [r for r in self.latest_laserscan.ranges if r > 0 and r < float('inf')]
        min_distance = min(valid_ranges) if valid_ranges else float('inf')
        self.obstacle_distance = min_distance

        self.obstacle_imminent_flag = False
        self.obstacle_near_flag = False

        if min_distance < self.obstacle_stop_threshold_:
            lin_vel = 0.0
            ang_vel = self.max_ang_vel_

            msg_cmd_vel = TwistStamped()
            msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            msg_cmd_vel.twist.linear.x = lin_vel
            msg_cmd_vel.twist.angular.z = ang_vel
            self.pub_cmd_vel_.publish(msg_cmd_vel)
            self.get_logger().info("hitting obstacle, proceed with re path planning")
            return
            



        lx = cos(-self.rbt_yaw_) * dx - sin(-self.rbt_yaw_) * dy
        ly = sin(-self.rbt_yaw_) * dx + cos(-self.rbt_yaw_) * dy
        
            # Calculate curvature (kappa) for pure pursuit
            # The basic formula is kappa = 2 * ly / Ld^2, where Ld is dist_to_lookahead
        if dist_to_lookahead > 0:
            curvature = 2 * ly / (dist_to_lookahead ** 2)
        else:
            curvature = 0.0

            # Calculate desired linear and angular velocities
            # Base linear speed with cap and gentle slowdown near target
        lin_vel = min(self.lookahead_lin_vel_, self.max_lin_vel_, dist_to_lookahead)
        ang_vel = curvature * lin_vel

            # Saturate angular velocity if needed
        ang_vel = max(-self.max_ang_vel_, min(self.max_ang_vel_, ang_vel))

            # saturate velocities. The following can result in the wrong curvature,
            # but only when the robot is travelling too fast (which should not occur if well tuned).
            #lin_vel = 0.0
            #ang_vel = 0.0 * lookahead_x * lookahead_y

        # publish velocities
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
