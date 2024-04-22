import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates, PacketPass
from sensor_msgs.msg import  Imu, JointState
from geometry_msgs.msg import Twist 


""" get up the robot by sending refernces interpolating from current to default pos """

class GetUp(Node):
    def __init__(self):
        super().__init__('getup_controller')

        # Topic names
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        self.joint_target_pos_topic     = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value

        self.simulation = False

        self.deltaT = 5.0
        self.rate = 100
        self.default_dof = np.array([
             2.094,     # flip
             -2.094,    # flip
             -2.094,
             2.094,
             -1.0472,   # flip
             1.0472,    # flip
             1.0472,
             -1.0472,
        ]) 

        # Initialize joint publisher/subscriber
        self.njoint = 8

        self.joint_names=(
            'LF_HFE',   # flip
            'LH_HFE',   # flip
            'RF_HFE',
            'RH_HFE',
            'LF_KFE',   # flip
            'LH_KFE',   # flip
            'RF_KFE',
            'RH_KFE',
        )

     
        self. i = 0

        self.init_pos = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joint_states')
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value

        if self.simulation:
            self.joint_state_topic = '/joint_states'
            self.joint_target_pos_topic = '/PD_control/command'
            self.joint_target_pos_pub = self.create_publisher(JointState, self.joint_target_pos_topic, 10)
            self.joint_sub  = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        else:
            self.joint_target_pos_pub = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)
            self.joint_sub  = self.create_subscription(JointsStates, self.joint_state_topic, self.joint_state_callback, 10)
        
        self.default_acquired = False

        rclpy.logging.get_logger('rclpy.node').info('GetUp started, waiting for joint state') 


    def getup_callback(self):
        WARMUP_ZONE = 200   
        start_pos = np.array(list(self.init_pos.values()))
        if self.i < WARMUP_ZONE:
            self.joint_pos =  start_pos
        else:
            # interpolate from current to default pos
            t = ((self.i - WARMUP_ZONE)/ (self.deltaT * self.rate))
            

            # linear interpolation:
            self.joint_pos = start_pos*(1-t) + self.default_dof*t


        rclpy.logging.get_logger('rclpy.node').info(f'joint pos: {self.joint_pos}') 
        rclpy.logging.get_logger('rclpy.node').info(f'i: {self.i}') 

        self.i += 1
        
        # SATURATE WARMPU_ZONE
        if self.i > WARMUP_ZONE +  self.deltaT * self.rate:
            self.i = WARMUP_ZONE +  self.deltaT * self.rate

        joint_msg = JointsCommand()
        if self.simulation:
            joint_msg = JointState()

        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = (self.joint_pos).tolist()
        joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        if not self.simulation:
            joint_msg.kp_scale = np.ones(self.njoint).tolist()
            joint_msg.kd_scale = np.ones(self.njoint).tolist()

        self.joint_target_pos_pub.publish(joint_msg)


    def joint_state_callback(self, msg):
        if self.default_acquired:
            return
        rclpy.logging.get_logger('rclpy.node').info('{}'.format((msg.position[:].tolist())))
        for i in range(self.njoint):
            self.init_pos[msg.name[i]] = msg.position[i]

        self.default_acquired = True
        rclpy.logging.get_logger('rclpy.node').info('Joints acquired, starting getup') 

        self.timer = self.create_timer(1.0 / self.rate, self.getup_callback)



def main(args=None):
    rclpy.init(args=args)
    getup = GetUp()
    rclpy.spin(getup)
    getup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
