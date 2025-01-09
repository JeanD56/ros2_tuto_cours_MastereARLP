import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
import transformations

class RobotKinematics(Node):
    def __init__(self):
        super().__init__('robot_kinematics')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.previous_time = None               
        self.L = 0.29                           #distance entre les roues
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

        # Modèle cinématique différentiel
        deltaX = self.R * (left_wheel_speed + right_wheel_speed) / 2.0
        deltaOmega = self.R * (right_wheel_speed - left_wheel_speed) / (2*self.L)

        # Mise à jour de la position
        #self.x += deltaX * math.cos(self.theta) * dt           #pas top tchatGPT
        #self.y += deltaX * math.sin(self.theta) * dt
        self.x = self.x + deltaX * math.cos(self.theta + deltaOmega)- deltaX * math.sin(self.theta + deltaOmega)
        self.y = self.y + deltaX * math.sin(self.theta + deltaOmega)+ deltaX * math.cos(self.theta + deltaOmega)
        self.theta = deltaOmega + self.theta

        #print(f"x: {self.x},    | y: {self.y},  | theta: {self.theta}")


        # Publier le tf
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'turtlebot_pos'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation._z = 0.0
        #t.transform.rotation =  self.theta #self.quaternion_from_euler(0, 0, self.theta)
        q = transformations.quaternion_about_axis(self.theta, [0, 0, 0])
        t.transform.rotation.x = 0 #q[0]
        t.transform.rotation.y = 0 #q[1]
        t.transform.rotation.z = q #q[2]
        t.transform.rotation.w = 1 #q[3]
        
        #print(f"tx: {t.transform.translation.x}, ty: {t.transform.translation.y}, tz: {t.transform.rotation.z}")

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def qzdqzdquaternion_from_euler(roll, pitch, yaw):
        """
        Convertit les angles d'Euler (roll, pitch, yaw) en quaternion (x, y, z, w).
        :param roll: Angle autour de l'axe X (en radians).
        :param pitch: Angle autour de l'axe Y (en radians).
        :param yaw: Angle autour de l'axe Z (en radians).
        :return: Tuple représentant le quaternion (x, y, z, w).
        """

        # Étape 1 : Calculer les demi-angles pour simplifier les formules
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0

        # Étape 2 : Calculer les sinus et cosinus des demi-angles
        sin_roll = math.sin(half_roll)
        cos_roll = math.cos(half_roll)

        sin_pitch = math.sin(half_pitch)
        cos_pitch = math.cos(half_pitch)

        sin_yaw = math.sin(half_yaw)
        cos_yaw = math.cos(half_yaw)

        # Étape 3 : Appliquer les formules pour chaque composante du quaternion
        qx = 0
        qy = 0
        qz = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
        qw = 1

        # Étape 4 : Retourner le quaternion sous forme de tuple
        return (qx, qy, qz, qw)


def main(args=None):
    print("Début du programme")
    rclpy.init(args=args)
    node = RobotKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
