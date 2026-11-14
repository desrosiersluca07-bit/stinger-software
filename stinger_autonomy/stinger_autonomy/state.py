'''
Stinger Finite State Machine
'''
import rclpy
import time  # ADDED
from rclpy.node import Node
from rclpy.duration import Duration  # ADDED
from geometry_msgs.msg import Twist
import numpy as np
from enum import Enum
from stinger_msgs.msg import Gate
from stinger_msgs.msg import Obstacle  # ADDED

class State(Enum):
    Searching = 0
    Approaching = 1
    Passing_Through = 2
    PassedThrough = 3
    Approaching_Goal = 4  # ADDED: drive to the yellow goal buoy after the gate

class StateMachine(Node):
    def __init__(self):
        super().__init__("State_Machine")

        self.image_width = 1280
        self.hfov = 1.09956
        self.angular_correction_factor = 1.0

        self.previous_state = None
        self.state = State.Searching

        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.pre_push_time = None # time difference from last push and now calls correct orientation state

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gate_pos_sub = self.create_subscription(Gate, '/stinger/gate_location', self.gate_callback, 10)
        self.obstacle_sub = self.create_subscription(Obstacle, '/stinger/obstacle', self.obstacle_callback, 10)  # ADDED: LiDAR obstacle info

        self.current_gate_pos : Gate = Gate()
        self.current_obstacle : Obstacle = Obstacle()  # ADDED
        self.obstacle_time = None  # ADDED: when the last obstacle msg arrived
        self.avoid_until = None  # ADDED: keep avoiding until this time
        self.avoid_turn = 1.0  # ADDED: +1 turn left, -1 turn right
        # TODO: tune these on the water
        self.avoid_speed = 0.05  # ADDED: forward speed while avoiding (m/s)
        self.avoid_turn_rate = 0.2  # ADDED: turn rate while avoiding (rad/s)
        self.avoid_hold = 2.0  # ADDED: seconds to keep avoiding after the obstacle leaves view
        self.goal_radius_px = 150  # ADDED: yellow buoy radius (pixels) that counts as reached
        self.get_logger().info("State Machine node initialized!")
    
    def gate_callback(self, msg: Gate):
        self.current_gate_pos = msg

    def obstacle_callback(self, msg: Obstacle):  # ADDED
        self.current_obstacle = msg
        self.obstacle_time = self.get_clock().now()

    def calculate_gate_fov_bound(self, left_gate_x, right_gate_x):
        distance = 1
        if left_gate_x is not None and right_gate_x is not None:
            distance = (right_gate_x - left_gate_x) / self.image_width
        return distance
    
    def state_debugger(self):
        if self.previous_state != self.state:
            self.get_logger().info(f"Now at: {self.state.name}")
            self.previous_state = self.state

    def state_machine_callback(self):
        self.state_debugger()
        cmd_vel = Twist()
        # ADDED: LiDAR avoidance overrides gate following while approaching the gate
        if self.state == State.Approaching:
            avoid_cmd = self.avoid()
            if avoid_cmd is not None:
                self.cmd_vel_pub.publish(avoid_cmd)
                return
        match self.state:
            case State.Searching:
                cmd_vel = self.search()
            case State.Approaching:
                cmd_vel = self.approach()
            case State.Passing_Through:
                cmd_vel = self.pass_through()
            case State.Approaching_Goal:  # ADDED
                cmd_vel = self.approach_goal()
            case State.PassedThrough:
                self.complete()
            case _:
                pass
        if cmd_vel is None:
            return
        self.cmd_vel_pub.publish(cmd_vel)

    def search(self):
        cmd_vel = Twist()
        left_gate_location = self.current_gate_pos.red_x
        right_gate_location = self.current_gate_pos.green_x

        self.get_logger().info(f"Red: {left_gate_location}, Green: {right_gate_location}")

        if left_gate_location < 0 or right_gate_location < 0:
            return
        
        deg_per_pixel = self.hfov / self.image_width

        mid_x = (right_gate_location + left_gate_location) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x

        '''
        If we are far too left, want to turn right, this will be a negative value
        If we are far too right, want to turn left, this will be a positive value
        '''
        turn_angle = diff_mid * deg_per_pixel

        self.get_logger().info(f"Turn angle: {turn_angle}")

        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1

        # TODO: 7.1.a Transition condition to move out of Search State
        ### STUDENT CODE HERE
        if abs(turn_angle) < 0.1:  
            self.state = State.Approaching
        ### END STUDENT CODE
        return cmd_vel
    
    def approach(self):
        cmd_vel = Twist()
        left_gate_location = self.current_gate_pos.red_x
        right_gate_location = self.current_gate_pos.green_x

        if left_gate_location < 0 or right_gate_location < 0:  # CHANGED: was 'is None', perception uses -1.0 for not seen
            return
        
        deg_per_pixel = self.hfov / self.image_width

        mid_x = (right_gate_location + left_gate_location) / 2
        mid_x_img = self.image_width // 2
        diff_mid = mid_x_img - mid_x
        turn_angle = diff_mid * deg_per_pixel

        cmd_vel.angular.z = self.angular_correction_factor * turn_angle
        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1
        cmd_vel.linear.x = 0.1
        gate_fov_bound = self.calculate_gate_fov_bound(left_gate_location, right_gate_location)
        
        self.get_logger().info(f"gate_fov_bound: {gate_fov_bound}")

        # to be between gate is to be done with the job

        # TODO: 7.1.b Transition condition to move out of Approach State
        ### STUDENT CODE HERE
        if abs(gate_fov_bound) > 0.5:#gate spans more than half image
            self.pre_push_time = self.get_clock().now()
            self.state = State.Passing_Through
        ### END STUDENT CODE

        return cmd_vel
    
    def pass_through(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        if (self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9 > 20:
            self.state = State.Approaching_Goal  # CHANGED: was State.PassedThrough, now go find the yellow buoy
        self.get_logger().info(f"{(self.get_clock().now() - self.pre_push_time).nanoseconds // 1e9}")
        return cmd_vel
    
    def avoid(self):  # ADDED: steer away from the obstacle the LiDAR node reports, None if nothing to avoid
        now = self.get_clock().now()
        fresh = self.obstacle_time is not None and (now - self.obstacle_time).nanoseconds / 1e9 < 1.0
        if fresh and self.current_obstacle.detected:
            # obstacle on the left (+angle) -> turn right, on the right -> turn left, dead ahead -> turn right
            self.avoid_turn = -1.0 if self.current_obstacle.angle >= 0.0 else 1.0
            self.avoid_until = now + Duration(seconds=self.avoid_hold)
        if self.avoid_until is None or now > self.avoid_until:
            return None
        cmd_vel = Twist()
        cmd_vel.linear.x = self.avoid_speed
        cmd_vel.angular.z = self.avoid_turn * self.avoid_turn_rate
        return cmd_vel

    def approach_goal(self):  # ADDED: steer at the yellow buoy, stop when it looks big enough
        cmd_vel = Twist()
        yellow_x = self.current_gate_pos.yellow_x
        yellow_radius = self.current_gate_pos.yellow_radius
        cmd_vel.linear.x = 0.1

        if yellow_x < 0:
            return cmd_vel  # goal buoy not visible, keep going straight

        deg_per_pixel = self.hfov / self.image_width
        diff_mid = (self.image_width // 2) - yellow_x
        turn_angle = diff_mid * deg_per_pixel

        cmd_vel.angular.z = self.angular_correction_factor * turn_angle
        if abs(cmd_vel.angular.z) > 0.1:
            cmd_vel.angular.z = np.sign(cmd_vel.angular.z) * 0.1

        self.get_logger().info(f"yellow_radius: {yellow_radius}")

        # TODO: tune goal_radius_px (how close counts as 'in front of the goal')
        if yellow_radius > self.goal_radius_px:
            self.get_logger().info("Goal buoy reached!")
            self.state = State.PassedThrough
            return Twist()
        return cmd_vel

    def complete(self):
        self.get_logger().info("Gate successfully passed!")
        self.cmd_vel_pub.publish(Twist())  # ADDED: zero cmd_vel so the tug stops
        time.sleep(0.5)  # ADDED: let the stop message go out before the node is destroyed
        self.destroy_node()

def main(args=None):  # ADDED: wrapped in main() so the state_node entry point works
    rclpy.init()
    state_machine = StateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # ADDED
    main()