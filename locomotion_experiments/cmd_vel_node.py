""" Ros2 node that publishes cmd_vel for experiments.
The node uses the vel_function(time), written by the user, to generate the cmd_vel
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time
import math



class CmdVelNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_node')
        #### parameters
        self.declare_parameter('publication_rate', 200) # Hz rate at which to publish cmd_vel
        self.declare_parameter('duration', 3.0)         # s net duration of the experiment
        self.declare_parameter('start_delay', 5.0)      # s delay before starting tp send ref
        self.declare_parameter('shutdown_delay', 2.0)   # s delay before shutting down the node
        self.declare_parameter('top_v', 0.5)            # vel      


        self.publication_rate = self.get_parameter('publication_rate').get_parameter_value().integer_value
        self.period_duration = self.get_parameter('period_duration').get_parameter_value().double_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.start_delay = self.get_parameter('start_delay').get_parameter_value().double_value
        self.shutdown_delay = self.get_parameter('shutdown_delay').get_parameter_value().double_value
        self.top_v = self.get_parameter('top_v').get_parameter_value().double_value


        # log parameters:
        self.get_logger().info('publication_rate: {}'.format(self.publication_rate))
        self.get_logger().info('duration: {}'.format(self.duration))
        self.get_logger().info('period_duration: {}'.format(self.period_duration))
        self.get_logger().info('start_delay: {}'.format(self.start_delay))
        self.get_logger().info('shutdown_delay: {}'.format(self.shutdown_delay))
        ####

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.startup_time = time.time()
        self.current_time = time.time()
        self.timer = self.create_timer(1.0 / self.publication_rate, self.timer_callback)
        self.get_logger().info('CmdVelNode started at {} s'.format(self.startup_time))

    def timer_callback(self):
        msg = Twist()
        t = time.time() - self.startup_time
        [vx, w] = self.vel_function(t)
        msg.linear.x = vx
        msg.angular.z = w  
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)


    def vel_function(self, t):
        """ Function that returns the desired velocity vector as a function of time.
        """

        vx = self.top_v
        w = 0.0
        # return non zero only if t is in the interval [start_delay, start_delay + duration]
        if t > self.start_delay and t < (self.start_delay + self.duration):
            te = t - self.start_delay
            
            # trapezoidal signal. 10% rise time
            # vxt = min(vx * te/(self.duration/10.0), vx, vx - vx*(te - 0.9*self.duration)*10.0/(self.duration))
            
            # ramp:
            # vxt = vx * te/self.duration
            
            # example: sinusoidal
            # vx = 0.3 * math.sin(2*math.pi*te/self.duration)
            
            # # Invert:
            # if t > (self.start_delay + self.duration/2.0):
            #     vx = -self.top_v

            # square wave
            vx = self.top_v * np.sign(np.sin(2*np.pi*te/self.period_duration))

            return [vx, w]
        elif t > (self.start_delay + self.duration + self.shutdown_delay):
            self.get_logger().info('CmdVelNode shutting down at {} s'.format(time.time()))
            self.destroy_timer(self.timer)
            self.destroy_node() # Is this safe? ¯\(°_o)/¯
            return [0.0, 0.0]
        else:
            return [0.0, 0.0] 
# /home/jacopo/Documents/locosim_ws/src/locomotion_experiments/locomotion_experiments/cmd_vel_node.py

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_node = CmdVelNode()
    rclpy.spin(cmd_vel_node)
    cmd_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main
