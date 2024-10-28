#Best Code 

import math
from std_msgs.msg import Int16
import rospy

class Inverse:
    l1 = 203
    l2 = 160.84
    l3 = 60
    l4 = 324
    l5 = 58
    l6 = 95.6
    lf = 98.09

    def InverseKinematicsFoot(self, x, y, z, alpha, beta):
        x1 = x + ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))
        x2 = x - ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))

        y1 = y + ((self.lf / 2) * math.cos(math.radians(beta)) * math.sin(math.radians(alpha)))
        y2 = y - ((self.lf / 2) * math.cos(math.radians(beta)) * math.sin(math.radians(alpha)))

        z1 = z + ((self.lf / 2) * math.sin(math.radians(beta)) * math.sin(math.radians(alpha)))
        z2 = z - ((self.lf / 2) * math.sin(math.radians(beta)) * math.sin(math.radians(alpha)))
        
        return x1, x2 , y1 , y2 , z1 , z2 
    
    #Hip Motor 1DOF
    def InverseKinematicsHip(self, y, z):
        thetaHip = math.atan2(y, z)
        return math.degrees(thetaHip)
    
    
    #Upper&Lower Motor 2DOF
    def InverseKinematics(self, x, z):
        # Calculate theta 2
        theta2_radians = math.acos((x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2))
        theta2_degrees = math.degrees(theta2_radians)

        # Calculate theta 1
        theta1_radians = math.atan((self.l2 * math.sin(theta2_radians)) / (self.l1 + self.l2 * math.cos(theta2_radians))) - math.atan(z/x) + math.pi / 2
        theta1_degrees = math.degrees(theta1_radians)

        # Calculate alpha 71 from triangle 1
        ct1_squared = self.l6**2 + self.l1**2 - 2 * self.l1 * self.l6 * math.cos(math.pi - theta1_radians)
        ct1 = math.sqrt(ct1_squared)

        alpha71_radians = math.acos((self.l6**2 + ct1**2 - self.l1**2) / (2 * self.l6 * ct1))
        alpha71_degrees = math.degrees(alpha71_radians)

        At1_radians = math.asin(self.l6 * (math.sin(alpha71_radians) / self.l1))
        At1_degrees = math.degrees(At1_radians)

        # Calculate alpha 72 from triangle 2
        ct2_squared = self.l3**2 + ct1**2 - 2 * self.l3 * ct1 * math.cos(math.pi - 2.11 + theta2_radians - At1_radians)
        ct2 = math.sqrt(ct2_squared)

        alpha72_radians = math.acos((ct1**2 + ct2**2 - self.l3**2) / (2 * ct1 * ct2))
        alpha72_degrees = math.degrees(alpha72_radians)

        # Calculate alpha 73 from triangle 3
        alpha73_radians = math.acos((self.l5**2 + ct2**2 - self.l4**2) / (2 * self.l5 * ct2))
        alpha73_degrees = math.degrees(alpha73_radians)

        # Calculate theta 3
        theta3_degrees = (alpha71_degrees + alpha72_degrees + alpha73_degrees)

        return theta1_degrees, theta2_degrees, theta3_degrees

def main():
    # Instantiate the Inverse class
    inv = Inverse()
    
    # Provide x and y values for testing
    x = 100 # still x
    z =  200 # previous Y
    y = 90 # added after Hip 
    alpha= 15
    beta = 31

    
    x1 , x2 , y1 , y2 , z1 , z2 = inv.InverseKinematicsFoot(x, y, z, alpha, beta)
    # Call the InverseKinematics method
    theta1r, theta2r, theta3r = inv.InverseKinematics(x1, z1)
    
    thetaHip = inv.InverseKinematicsHip(y1, z1)

    theta1l, theta2l, theta3l = inv.InverseKinematics(x2, z2)

    # Print the results right 
    print(f"Theta 1 Lower motor Right: {theta1r} degrees")
    print(f"Theta 2 knee Right: {theta2r} degrees")
    print(f"Theta 3 Upper motor Right: {theta3r} degrees")
    print(f"Theta Hip Upper motor : {thetaHip} degrees")
    
    theta1shiftedr = theta1r + 180
    theta3shiftedr = theta3r + 27 
    thetaHipshifted = thetaHip + 180 

    print(f"Theta1 shifted Lower motor Right: {theta1shiftedr} degrees")
    print(f"Theta2 shifted knee Right: {theta2r} degrees")
    print(f"Theta3 shifted Upper motor Right: {theta3shiftedr} degrees")
    print(f"Theta Hip shifted : {thetaHipshifted} degrees")

    print(f"Theta1 shifted Lower motor Right decimal: {int(theta1shiftedr / 360 * 4096)} degrees")
    print(f"Theta3 shifted Upper motor Right decimal: {int(theta3shiftedr / 360 * 4096)} degrees")
    print(f"TTheta Hip shifted decimal: {int(thetaHipshifted / 360 * 4096)} degrees")

     # Print the results Left
    print(f"Theta 1 Lower motor Left: {theta1l} degrees")
    print(f"Theta 2 knee Left: {theta2l} degrees")
    print(f"Theta 3 Upper motor Left: {theta3l} degrees")
    print(f"Theta Hip Upper motor : {thetaHip} degrees")
    
    theta1shiftedl = theta1r + 180
    theta3shiftedl = theta3r + 27 

    print(f"Theta1 shifted Lower motor Left: {theta1shiftedl} degrees")
    print(f"Theta2 shifted knee Left: {theta2r} degrees")
    print(f"Theta3 shifted Upper motor Left: {theta3shiftedl} degrees")
    print(f"Theta Hip shifted : {thetaHipshifted} degrees")

    print(f"Theta1 shifted Lower motor Left decimal: {int(theta1shiftedl / 360 * 4096)} degrees")
    print(f"Theta3 shifted Upper motor Left decimal: {int(theta3shiftedl / 360 * 4096)} degrees")
    print(f"TTheta Hip shifted decimal: {int(thetaHipshifted / 360 * 4096)} degrees")
    
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshifted, theta1shiftedl, theta2l, theta3shiftedl

def moveMotor():
    rospy.init_node('moveMotor', anonymous=True)
    
    pubTh1r14 = rospy.Publisher('r14', Int16, queue_size=10)
    pubTh3r13 = rospy.Publisher('r13', Int16, queue_size=10)
    pubThHipr15 = rospy.Publisher('r15', Int16, queue_size=10)
    pubTh1r12 = rospy.Publisher('r12', Int16, queue_size=10)
    pubTh3r11 = rospy.Publisher('r11', Int16, queue_size=10)

    theta1shiftedr, theta2r, theta3shiftedr, thetaHipshifted, theta1shiftedl, theta2l, theta3shiftedl= main()
    
    # Convert to motor-specific range
    motor_pos_Th1r14 = int(theta1shiftedr / 360 * 4096)
    motor_pos_Th3r13 = int(theta3shiftedr / 360 * 4096)
    motor_pos_ThHipr15 = int(thetaHipshifted / 360 * 4096)
    motor_pos_Th1r12 = int(theta1shiftedl / 360 * 4096)
    motor_pos_Th3r11 = int(theta3shiftedl / 360 * 4096)

    rospy.sleep(1)  # Allow time for publisher to connect
    
    rospy.loginfo(f"Publishing to r14: {motor_pos_Th1r14}")
    rospy.loginfo(f"Publishing to r13: {motor_pos_Th3r13}")
    rospy.loginfo(f"Publishing to r15: {motor_pos_ThHipr15}")
    rospy.loginfo(f"Publishing to r12: {motor_pos_Th1r12}")
    rospy.loginfo(f"Publishing to r11: {motor_pos_Th3r11}")

    pubTh1r14.publish(motor_pos_Th1r14)
    pubTh3r13.publish(motor_pos_Th3r13)
    pubThHipr15.publish(motor_pos_ThHipr15)
    pubTh1r12.publish(motor_pos_Th1r12)
    pubTh3r11.publish(motor_pos_Th3r11)
    

    rospy.sleep(1)  # Give time for messages to be sent

if __name__ == "__main__":
    try:
        moveMotor()
    except rospy.ROSInterruptException:
        pass
