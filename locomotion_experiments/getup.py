import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates, PacketPass
from sensor_msgs.msg import  Imu, JointState
from geometry_msgs.msg import Twist 


""" get up the robot by sending refernces interpolating fro 0 to default pos """

class GetUp(Node):
    def __init__(self):
        super().__init__('getup_controller')

        with open(r'src/filc_controller/config/MULINEX_traj_opt_abs_theta=13_SEA_no_dampers_obj_CoT.yaml') as file:
            data = yaml.safe_load(file)     

        x_ref = np.array(data['x'])
        z_ref = np.array(data['z'])
        alpha_ref = -np.array(data['alpha'])
        x_dot_ref = np.array(data['x_dot'])
        z_dot_ref = np.array(data['z_dot'])
        alpha_dot_ref = -np.array(data['alpha_dot'])
        theta_2 = np.array(data['theta_2'])[:-1]
        theta_3 = np.array(data['theta_3'])[:-1]
        theta_4 = np.array(data['theta_4'])[:-1]
        theta_5 = np.array(data['theta_5'])[:-1]
        theta_2_dot = np.array(data['theta_2_dot'])[:-1]
        theta_3_dot = np.array(data['theta_3_dot'])[:-1]
        theta_4_dot = np.array(data['theta_4_dot'])[:-1]
        theta_5_dot = np.array(data['theta_5_dot'])[:-1]

        self.default_dof    = (
        theta_4[0]*-1, 
        (np.pi + theta_3[0])*-1, 
        theta_4[0], 
        (np.pi + theta_3[0]),
        -theta_5[0],
        -theta_2[0],
        theta_5[0],
        theta_2[0]
        )


        self.declare_parameters(
            namespace='',
            parameters=[
                ('joint_state_topic', rclpy.Parameter.Type.STRING),
                ('joint_target_torque_topic', rclpy.Parameter.Type.STRING),
                ('cmd_vel_topic', rclpy.Parameter.Type.STRING),
                ('imu_topic', rclpy.Parameter.Type.STRING),
                ('rate', rclpy.Parameter.Type.INTEGER),
                ('use_imu', rclpy.Parameter.Type.BOOL),
                ('imu_topic', rclpy.Parameter.Type.STRING),
                ('simulation', rclpy.Parameter.Type.BOOL),  
                ('traj_gen_joint_reference_topic', rclpy.Parameter.Type.STRING),
                ('njoint', rclpy.Parameter.Type.INTEGER), 
                ('default_hip_angle', rclpy.Parameter.Type.INTEGER), 
                ('default_knee_angle', rclpy.Parameter.Type.INTEGER),
                ('joint_names', rclpy.Parameter.Type.STRING_ARRAY),  
                ('T_idx', rclpy.Parameter.Type.INTEGER_ARRAY),  
                ('n_iter', rclpy.Parameter.Type.INTEGER),  
                ('variance', rclpy.Parameter.Type.DOUBLE), 
                ('Q', rclpy.Parameter.Type.DOUBLE), 
                ('S', rclpy.Parameter.Type.DOUBLE),  
                ('scaling', rclpy.Parameter.Type.DOUBLE_ARRAY),              
                ('k_f', rclpy.Parameter.Type.DOUBLE), 
                ('initial_hip_angle', rclpy.Parameter.Type.INTEGER), 
                ('initial_knee_angle', rclpy.Parameter.Type.INTEGER),                 
            ]
        )

        self.joint_target_torque_topic = self.get_parameter('joint_target_torque_topic').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.njoint = self.get_parameter('njoint').get_parameter_value().integer_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value        


        self.deltaT = 5.0
        
        self.joint_target_pos_pub = self.create_publisher(JointsCommand, self.joint_target_torque_topic, 10)
        self.i = 0
        self.joint_pos = np.zeros((self.njoint, 1))
        self.joint_vel = {self.joint_names[i]:0.0 for i in range(self.njoint)}
   
        self.timer = self.create_timer(1.0 / self.rate, self.getup_callback)
        rclpy.logging.get_logger('rclpy.node').info('Getting Up.') 


    def getup_callback(self):
        # interpolate from zero to default pos
        self.joint_pos = self.default_dof * (self.i / (self.deltaT * self.rate))
        self.i += 1
        if self.i > self.deltaT * self.rate:
            self.i = self.deltaT * self.rate

        joint_msg = JointsCommand()
        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = (self.joint_pos).tolist()
        joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        joint_msg.kp_scale = np.ones(self.njoint).tolist()
        joint_msg.kd_scale = np.ones(self.njoint).tolist()

        self.joint_target_pos_pub.publish(joint_msg)



def main(args=None):
    rclpy.init(args=args)
    getup = GetUp()
    rclpy.spin(getup)
    getup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
