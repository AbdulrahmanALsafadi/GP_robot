import math
import numpy as np

class Robot6DOF:
    def __init__(self, l2, l3, l4, l5, d6):
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.l5 = l5
        self.d6 = d6

    def inverse_kinematics(self, PE):
        """
        Calculate inverse kinematics for the 6-DOF robot.
        
        :param PE: End-effector position [x, y, z]
        :return: Joint angles [theta1, theta2, theta3, theta4, theta5, theta6]
        """
        x, y, z = PE
        
        # Calculate r1
        r1 = math.sqrt(x**2 + y**2 + z**2)
        
        # Calculate theta1
        theta1 = math.atan2(-x, y)
        
        # Calculate theta2
        theta2 = math.atan2(z, y) - math.acos((self.l2**2 + r1**2 - self.l3**2) / (2 * self.l2 * r1))
        
        # Calculate theta3
        theta3 = math.pi - math.acos((self.l2**2 + self.l3**2 - r1**2) / (2 * self.l2 * self.l3))
        
        # Calculate V3 and V2
        V3 = [x - self.l4 * math.cos(theta1) * math.sin(theta2 + theta3),
              y - self.l4 * math.sin(theta1) * math.sin(theta2 + theta3),
              z - self.l4 * math.cos(theta2 + theta3)]
        
        V2 = [x - self.l4 * math.cos(theta1) * math.sin(theta2),
              y - self.l4 * math.sin(theta1) * math.sin(theta2),
              z - self.l4 * math.cos(theta2)]
        
        V3_mag = np.linalg.norm(V3)
        V2_mag = np.linalg.norm(V2)
        
        # Calculate beta1, beta2, beta3
        beta1 = math.acos((self.l4**2 + V3_mag**2 - self.l5**2) / (2 * self.l4 * V3_mag))
        beta2 = math.acos((V2_mag**2 + V3_mag**2 - self.d6**2) / (2 * V2_mag * V3_mag))
        beta3 = math.acos((V2_mag**2 + self.l4**2 - self.l5**2) / (2 * self.l4 * V2_mag))
        
        # Calculate theta4
        theta4 = math.pi - (beta1 + beta2 + beta3)
        
        # For simplicity, we'll set theta5 and theta6 to 0
        # In a real application, these would be calculated based on the desired orientation
        theta5 = 0
        theta6 = 0
        
        return [theta1, theta2, theta3, theta4, theta5, theta6]

# Example usage
if __name__ == "__main__":
    # Create a robot with example link lengths
    robot = Robot6DOF(l2=10, l3=10, l4=5, l5=5, d6=2)
    
    # Test the inverse kinematics with an example end-effector position
    end_effector_position = [5, 5, 5]
    joint_angles = robot.inverse_kinematics(end_effector_position)
    
    print("End-effector position:", end_effector_position)
    print("Calculated joint angles (in radians):")
    for i, angle in enumerate(joint_angles, 1):
        print(f"theta{i}: {angle:.4f}")
