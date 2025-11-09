from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from math import hypot, atan2, sin, cos, pi
from sensor_msgs.msg import LaserScan


print("omg it finally worked")

class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)


        self.goal_reached_ = True
        self.received_lookahead_ = False
        self.latest_laserscan_ = None


        # Parameters: Declare
        self.declare_parameter("frequency", float(10))
        self.declare_parameter("plan_frequency", float(2))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.plan_frequency_ = self.get_parameter("plan_frequency").value

        #below is for unstuck mission 
        self.prev_rbt_x_ = None
        self.prev_rbt_y_ = None
        self.stuck_counter_ = 0
        self.stuck_threshold_ = 20   # number of timer ticks to consider stuck
        self.stuck_distance_ = 0.01  # minimum distance to consider not moving
        self.retreat_distance_ = 0.2 # how far to retreat
        self.obstacle_slowdown_threshold_ = 0.5
        self.obstacle_stop_threshold_ = 0.1

           
        self.stuck_recovery_mode_ = False
        self.spinspin_mode_ = False
        self.obstacle_avoidance_mode_ = False
        

        # Handles: Topic Subscribers
        # !TODO: Goal pose subscriber
        	
        self.goal_pose = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.callbackSubGoalPose_,
            qos_profile_services_default#use this for reliable data, every data must be kept
        )

        # !TODO: Odometry subscriber
        self.odometry = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            qos_profile_services_default #use this for reliable data, every data must be kept
        )



        self.sub_lookahead_ = self.create_subscription(
            PoseStamped,
            "lookahead_point",
            self.callbackSubLookahead_,
            qos_profile_services_default
        )
        
        self.sub_laserscan_ = self.create_subscription(
            LaserScan, 
            "scan", 
            self.callbackSubLaserScan_, 
            qos_profile_sensor_data
        )

        # Handles: Topic Publishers
        # !TODO: Path request publisher
        self.pub_path_request_ = self.create_publisher(
            Path,
            "path_request",
            qos_profile_services_default
        )   



        self.pub_stuck_recovery_mode_flag_ = self.create_publisher(
            Bool,
            "stuck_recovery_mode",
            qos_profile_services_default
        )

        self.pub_spinspin_mode_flag_ = self.create_publisher(
            Bool,
            "spinspin_mode",
            qos_profile_services_default
        )

        self.pub_obstacle_avoidance_mode_flag_ = self.create_publisher(
            Bool,
            "obstacle_avoidance_mode",
            qos_profile_services_default
        )


        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        self.timer_plan_ = self.create_timer(
            1.0 / self.plan_frequency_, self.callbackTimerPlan_
        )

        # Other Instance Variables
        self.received_goal_coords_ = False
        self.received_rbt_coords_ = False
        self.goal_reached_ = True

    # Callbacks =============================================================

    # Goal pose subscriber callback.
    def callbackSubGoalPose_(self, msg: PoseStamped):
        self.received_goal_coords_ = True
        self.goal_reached_ = (
            True  # "cancel" the goal and trigger the if condition in timer.
        )

        # !TODO: Copy to goal_x_, goal_y_.
        self.goal_x_ = msg.pose.position.x
        self.goal_y_ = msg.pose.position.y

        self.get_logger().info(
            f"Received New Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
        )

    # Odometry subscriber callback.
    def callbackSubOdom_(self, msg: Odometry):
        self.received_rbt_coords_ = True

        # !TODO: Copy to rbt_x_, rbt_y_.
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y
        
        # Get yaw from quaternion
        q = msg.pose.pose.orientation
        self.rbt_yaw_ = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


    def callbackSubLookahead_(self, msg: PoseStamped):
        self.lookahead_x_ = msg.pose.position.x
        self.lookahead_y_ = msg.pose.position.y
        self.received_lookahead_ = True

    def callbackSubLaserScan_(self, msg: LaserScan):
        self.latest_laserscan_ = msg  # Note the underscore at the end


    # Callback for timer.
    # Normally the decisions of the robot system are made here, and this callback is dramatically simplified.
    # The callback contains some example code for waypoint detection.
    def getLookaheadPoint_(self):
        if not hasattr(self, 'lookahead_x_') or not hasattr(self, 'lookahead_y_'):
            return self.goal_x_, self.goal_y_  
        return self.lookahead_x_, self.lookahead_y_

    def callbackTimer_(self):
        if not self.received_rbt_coords_ or not self.received_goal_coords_:
            return  # silently return if none of the coords are received from the subscribers.

        dx = self.goal_x_ - self.rbt_x_
        dy = self.goal_y_ - self.rbt_y_

        goal_is_close = hypot(dx, dy) < 0.1

        if goal_is_close and not self.goal_reached_:
            self.get_logger().info(
                f"Reached Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = True
        elif not goal_is_close and self.goal_reached_:
            self.get_logger().info(
                f"Going to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})."
            )
            self.goal_reached_ = False

    #stuck recovery mode
        if self.prev_rbt_x_ is not None:
            dist_moved = hypot(self.rbt_x_ - self.prev_rbt_x_, self.rbt_y_ - self.prev_rbt_y_)
            if dist_moved < self.stuck_distance_ and not goal_is_close:
                self.stuck_counter_ += 1
            else:
                self.stuck_counter_ = 0

        if self.stuck_counter_ == 0:
            self.stuck_recovery_mode_ = False
            self.spinspin_mode_ = False
            self.obstacle_avoidance_mode_ = False       

        # If stuck too long, trigger retreat
        if self.stuck_counter_ >= self.stuck_threshold_ and not self.stuck_recovery_mode_:
            self.stuck_recovery_mode_ = True
            self.stuck_counter_ = 0
            self.get_logger().info("=================< STUCK RECOVERINGG >==========================")

    #spinspin mode 
        lookahead_x, lookahead_y = self.getLookaheadPoint_()
        
        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        
        def shortest_angle_diff(a, b):
            # return signed smallest angle from b --> a, in [-pi, pi]
            d = a - b
            return atan2(sin(d), cos(d))

        angle_to_point = atan2(dy, dx)
        angle_diff = shortest_angle_diff(angle_to_point, self.rbt_yaw_)

        # pi/2 is 90 degrees, 0.87 is about 50 degrees , 0.26 is about 15 degrees
        if abs(angle_diff) > 0.87 and abs(angle_diff) < pi:
            self.spinspin_mode_ = True
            self.get_logger().info("SPINSPINSPINSPINSPINSPINSPINSPINSPINSPINSPINSPINSPINSPIN")
        else:
            self.spinspin_mode_ = False

    #Obstacle Avoidance Mode 
        if self.latest_laserscan_:
            valid_ranges = [r for r in self.latest_laserscan_.ranges if r > 0.0 and r < float('inf')]
            if valid_ranges:
                min_dist = min(valid_ranges)
            else:
                min_dist = float('inf')

            if min_dist < self.obstacle_stop_threshold_:
                if not self.obstacle_avoidance_mode_:
                    self.get_logger().warn(f" OBSTACLE DETECTED — STOPPING (dist = {min_dist:.2f} m)")
                self.obstacle_avoidance_mode_ = True
            elif min_dist < self.obstacle_slowdown_threshold_:
                # if not self.obstacle_avoidance_mode_:
                #     self.get_logger().info(f" OBSTACLE NEARBY — SLOWDOWN (dist = {min_dist:.2f} m)")
                self.obstacle_avoidance_mode_ = False
            else:
                self.obstacle_avoidance_mode_ = False



        stuck_recovery_mode_msg = Bool()
        stuck_recovery_mode_msg.data = self.stuck_recovery_mode_
        self.pub_stuck_recovery_mode_flag_.publish(stuck_recovery_mode_msg)

        spinspin_mode_msg = Bool()
        spinspin_mode_msg.data = self.spinspin_mode_
        self.pub_spinspin_mode_flag_.publish(spinspin_mode_msg)

        obstacle_avoidance_mode_msg = Bool()
        obstacle_avoidance_mode_msg.data = self.obstacle_avoidance_mode_
        self.pub_obstacle_avoidance_mode_flag_.publish(obstacle_avoidance_mode_msg)

        # Limit spamming
        if self.get_clock().now().nanoseconds % 20 == 0:
            self.get_logger().info(f"Flags -> STUCK:{self.stuck_recovery_mode_} | SPIN:{self.spinspin_mode_} | OBSTACLE:{self.obstacle_avoidance_mode_}")

        # Update previous robot pose
        self.prev_rbt_x_ = self.rbt_x_
        self.prev_rbt_y_ = self.rbt_y_

    # Callback for publishing path requests between clicked_point (goal) and robot position.
    # Normally path requests are implemented with ROS2 service, and the service is called in the main timer.
    # To keep things simple for this course, we use only ROS2 tpics.
    def callbackTimerPlan_(self):
        if not self.received_goal_coords_ or not self.received_rbt_coords_:
            return  # silently return if none of the coords are received from the subscribers

        # Create a new message for publishing
        msg_path_request = Path()
        msg_path_request.header.stamp = self.get_clock().now().to_msg()
        msg_path_request.header.frame_id = "map"

        # !TODO: write the robot coordinates
        rbt_pose = PoseStamped()
        rbt_pose.pose.position.x = self.rbt_x_
        rbt_pose.pose.position.y = self.rbt_y_
        rbt_pose.header.frame_id = "map"
        rbt_pose.header.stamp = self.get_clock().now().to_msg()



        # !TODO: write the goal coordinates
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.goal_x_
        goal_pose.pose.position.y = self.goal_y_
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # !TODO: fill up the array containing the robot coordinates at [0] and goal coordinates at [1]
        msg_path_request.poses.clear()  #clear it to ensure the msg is appended to [0] and [1] only
        msg_path_request.poses.append(rbt_pose)
        msg_path_request.poses.append(goal_pose)

        # publish the message
        self.get_logger().info(
            f"Sending Path Planning Request from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f}, {self.goal_y_:7.3f})"
        )
        self.pub_path_request_.publish(msg_path_request)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
