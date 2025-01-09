import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
from geometry_msgs.msg import Quaternion

class RobotKinematics(Node):
    def __init__(self):
        super().__init__('robot_kinematics')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.previous_time = None               
        self.L = 0.288                          #distance entre les roues
        self.R = 0.033                          #rayon des roues

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        if self.previous_time is None:
            self.previous_time = current_time
            return

        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        self.previous_time = current_time

        # Récupération des vitesses angulaires des roues
        left_wheel_speed = msg.velocity[0]
        right_wheel_speed = msg.velocity[1]

        #left_wheel_speed = left_wheel_speed_rpm * (2 * math.pi / 60)
        #right_wheel_speed = right_wheel_speed_rpm * (2 * math.pi / 60)


        # Modèle cinématique différentiel
        deltaX = self.R * (left_wheel_speed + right_wheel_speed) / 2.0 *dt
        deltaOmega = self.R * (right_wheel_speed - left_wheel_speed) / (2*self.L) *dt

        # Mise à jour de la position
        #self.x += deltaX * math.cos(self.theta) * dt           #pas top tchatGPT
        #self.y += deltaX * math.sin(self.theta) * dt
        self.theta = deltaOmega + self.theta
        self.x += deltaX * math.cos(self.theta)
        self.y += deltaX * math.sin(self.theta)

        #print(f"x: {self.x},    | y: {self.y},  | theta: {self.theta}")


        # Publier le tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'turtlebot_pos'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        #t.transform.rotation =  self.theta #self.quaternion_from_euler(0, 0, self.theta)
        q = self.euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        
        #print(f"tx: {t.transform.translation.x}, ty: {t.transform.translation.y}, tz: {t.transform.rotation.z}")

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convertit les angles d'Euler (roll, pitch, yaw) en quaternion.
        
        :param roll: Rotation autour de l'axe X (en radians)
        :param pitch: Rotation autour de l'axe Y (en radians)
        :param yaw: Rotation autour de l'axe Z (en radians)
        :return: Un objet Quaternion contenant les composantes (x, y, z, w)
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    print("Début du programme")
    rclpy.init(args=args)
    node = RobotKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
