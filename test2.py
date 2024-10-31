import math
import rospy
from std_msgs.msg import Int16

class Inverse:
    l1 = 203
    l2 = 160.84
    l3 = 60
    l4 = 324
    l5 = 58
    l6 = 95.6
    lf = 98 

    def InverseKinematicsFoot(self, x, y, z, alpha, beta):
        xA = x - (((self.lf) / 2) * math.cos(math.radians(beta)) * math.sin(math.radians(alpha)))
        xB = x + (((self.lf) / 2) * math.cos(math.radians(beta)) * math.sin(math.radians(alpha)))

        yA = y + ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))
        yB = y - ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))

        zA = z + ((self.lf / 2) * math.sin(math.radians(beta)) * math.cos(math.radians(alpha)))
        zB = z - ((self.lf / 2) * math.sin(math.radians(beta)) * math.cos(math.radians(alpha)))

        zA = abs(zA)
        zB = abs(zB)

        return xA, xB, yA, yB, zA, zB 
    
    def InverseKinematicsHip(self, y, z):
        thetaHip = math.atan2(165 + y, z)
        return math.degrees(thetaHip)

    def InverseKinematicsHip2(self, y, z):
        thetaHip = math.atan2(165 - y, z)
        return math.degrees(thetaHip)
    
    def InverseKinematics(self, x, z):
        cos_theta2 = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1, min(1, cos_theta2))
        theta2_radians = math.acos(cos_theta2)
        theta2 = math.degrees(theta2_radians)

        theta1_radians = math.atan((self.l2 * math.sin(theta2_radians)) / (self.l1 + self.l2 * math.cos(theta2_radians))) - math.atan2(z, x) + math.pi / 2
        theta1 = math.degrees(theta1_radians)

        ct1_squared = self.l6**2 + self.l1**2 - 2 * self.l1 * self.l6 * math.cos(math.pi - theta1_radians)
        ct1 = math.sqrt(ct1_squared)

        cos_alpha71 = (self.l6**2 + ct1**2 - self.l1**2) / (2 * self.l6 * ct1)
        cos_alpha71 = max(-1, min(1, cos_alpha71))
        alpha71_radians = math.acos(cos_alpha71)
        alpha71_degrees = math.degrees(alpha71_radians)

        sin_At1 = self.l6 * (math.sin(alpha71_radians) / self.l1)
        sin_At1 = max(-1, min(1, sin_At1))
        At1_radians = math.asin(sin_At1)
        At1_degrees = math.degrees(At1_radians)

        ct2_squared = self.l3**2 + ct1**2 - 2 * self.l3 * ct1 * math.cos(math.pi - 2.11 + theta2_radians - At1_radians)
        ct2 = math.sqrt(ct2_squared)

        cos_alpha72 = (ct1**2 + ct2**2 - self.l3**2) / (2 * ct1 * ct2)
        cos_alpha72 = max(-1, min(1, cos_alpha72))
        alpha72_radians = math.acos(cos_alpha72)
        alpha72_degrees = math.degrees(alpha72_radians)

        cos_alpha73 = (self.l5**2 + ct2**2 - self.l4**2) / (2 * self.l5 * ct2)
        cos_alpha73 = max(-1, min(1, cos_alpha73))
        alpha73_radians = math.acos(cos_alpha73)
        alpha73_degrees = math.degrees(alpha73_radians)

        theta3 = (alpha71_degrees + alpha72_degrees + alpha73_degrees)

        return theta1, theta2, theta3

def mainRight():
    inv = Inverse()
    print("Enter the value of Right leg:")
    xr, zr, yr, alphar, betar, gammar = map(float, input("Enter values for xr, zr, yr, alphar, betar, gammar (separated by spaces): ").split())

    xAr, xBr, yAr, yBr, zAr, zBr = inv.InverseKinematicsFoot(xr, yr, zr, alphar, betar)
    theta1r, theta2r, theta3r = inv.InverseKinematics(xBr, zBr)
    thetaHipr = inv.InverseKinematicsHip(yBr, zBr)

    theta1l, theta2l, theta3l = inv.InverseKinematics(xAr, zAr)
    thetaHipl = inv.InverseKinematicsHip(yAr, zAr)

    # Shift angles
    theta1shiftedr = theta1r + 180
    theta3shiftedr = theta3r + 27 
    thetaHipshiftedr = thetaHipr + 180 

    theta1shiftedl = theta1l + 180
    theta3shiftedl = theta3l + 27 
    thetaHipshiftedl = thetaHipl + 180 
    
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl, thetaHipshiftedl, gammar

def mainLeft():   
    inv = Inverse()
    print("Enter the value of Left leg:")
    xl, zl, yl, alphal, betal, gammal = map(float, input("Enter values for xl, zl, yl, alphal, betal, gammal (separated by spaces): ").split())
    
    xAl, xBl, yAl, yBl, zAl, zBl = inv.InverseKinematicsFoot(xl, yl, zl, alphal, betal)
    
    theta1r, theta2r, theta3r = inv.InverseKinematics(xAl, zAl)
    thetaHipr = inv.InverseKinematicsHip2(yAl, zAl)

    theta1l, theta2l, theta3l = inv.InverseKinematics(xBl, zBl)
    thetaHipl = inv.InverseKinematicsHip2(yBl, zBl)

    # Shift angles
    theta1shiftedr = theta1r + 180
    theta3shiftedr = theta3r + 27 
    thetaHipshiftedr = thetaHipr + 180 

    theta1shiftedl = theta1l + 180
    theta3shiftedl = theta3l + 27 
    thetaHipshiftedl = thetaHipl + 180 
    
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl, thetaHipshiftedl, gammal

def smooth_trajectory(start_pos, end_pos, steps=50):
    """
    Generate smooth trajectory points between start and end positions
    """
    trajectory = []
    for step in range(steps + 1):
        t = step / steps
        smooth_t = t * t * (3 - 2 * t)  
        pos = start_pos + (end_pos - start_pos) * smooth_t
        trajectory.append(pos)
    return trajectory

def moveMotor():
    rospy.init_node('moveMotor', anonymous=True)
    
    # Create publishers
    pubTh1r14 = rospy.Publisher('r14', Int16, queue_size=10)
    pubTh3r13 = rospy.Publisher('r13', Int16, queue_size=10)
    pubThHipr15 = rospy.Publisher('r15', Int16, queue_size=10)
    
    pubTh1r24 = rospy.Publisher('r24', Int16, queue_size=10)
    pubTh3r23 = rospy.Publisher('r23', Int16, queue_size=10)
    pubThHipr25 = rospy.Publisher('r25', Int16, queue_size=10)

    # Get motor positions for both legs
    theta1A, theta2A, theta3A, thetaHipA, theta1B, theta2B, theta3B, thetaHipB, gammar = mainRight()
    theta1C, theta2C, theta3C, thetaHipC, theta1D, theta2D, theta3D, thetaHipD, gammal = mainLeft()

    # Convert angles to motor positions (0-4096)
    pos1A = int(theta1A / 360 * 4096)
    pos3A = int(theta3A / 360 * 4096)
    posHipA = int(thetaHipA / 360 * 4096)
    
    pos1B = int(theta1B / 360 * 4096)
    pos3B = int(theta3B / 360 * 4096)
    posHipB = int(thetaHipB / 360 * 4096)

    # Define current positions (you need to implement a way to get these)
    current_pos1A = 2048  # Replace with actual current position
    current_pos3A = 2048  # Replace with actual current position
    current_posHipA = 2048  # Replace with actual current position
    current_pos1B = 2048  # Replace with actual current position
    current_pos3B = 2048  # Replace with actual current position
    current_posHipB = 2048  # Replace with actual current position

    # Generate smooth trajectories
    steps = 50  # Number of intermediate points
    delay = 0.02  # Delay between points in seconds

    # Generate trajectories for each motor
    traj1A = smooth_trajectory(current_pos1A, pos1A, steps)
    traj3A = smooth_trajectory(current_pos3A, pos3A, steps)
    trajHipA = smooth_trajectory(current_posHipA, posHipA, steps)
    traj1B = smooth_trajectory(current_pos1B, pos1B, steps)
    traj3B = smooth_trajectory(current_pos3B, pos3B, steps)
    trajHipB = smooth_trajectory(current_posHipB, posHipB, steps)

    # Execute smooth movement
    for i in range(len(traj1A)):
        # Publish positions for all motors
        pubTh1r14.publish(Int16(traj1A[i]))
        pubTh3r13.publish(Int16(traj3A[i]))
        pubThHipr15.publish(Int16(trajHipA[i]))
        
        pubTh1r24.publish(Int16(traj1B[i]))
        pubTh3r23.publish(Int16(traj3B[i]))
        pubThHipr25.publish(Int16(trajHipB[i]))
        
        rospy.sleep(delay)

    rospy.sleep(1)  # Give time for final positions to be reached

if __name__ == "__main__":
    try:
        moveMotor()
    except rospy.ROSInterruptException:
        pass
