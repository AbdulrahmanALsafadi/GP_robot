#Best Code 

import math
from std_msgs.msg import Int16
import rospy
from sensor_msgs.msg import JointState
from motor_postion2 import MotorHandler


class MoveHandler:
    @staticmethod
    def move_motor(motor_name, goal_position):
        # Dictionary to store motor publishers
        motor_publishers = {
            'r14': rospy.Publisher('r14', Int16, queue_size=10),
            'r13': rospy.Publisher('r13', Int16, queue_size=10),
            'r15': rospy.Publisher('r15', Int16, queue_size=10),
            'r12': rospy.Publisher('r12', Int16, queue_size=10),
            'r11': rospy.Publisher('r11', Int16, queue_size=10),
            'r16': rospy.Publisher('r16', Int16, queue_size=10),
            'r24': rospy.Publisher('r24', Int16, queue_size=10),
            'r23': rospy.Publisher('r23', Int16, queue_size=10),
            'r25': rospy.Publisher('r25', Int16, queue_size=10),
            'r22': rospy.Publisher('r22', Int16, queue_size=10),
            'r21': rospy.Publisher('r21', Int16, queue_size=10),
            'r26': rospy.Publisher('r26', Int16, queue_size=10)
        }

        if motor_name not in motor_publishers:
            rospy.logerr(f"Motor name '{motor_name}' is not valid.")
            return

        motor_pos = int(goal_position)  # Convert goal position to motor-specific range

        rospy.sleep(0.000001)  # Allow time for publisher to connect

        # Log and publish the position
        rospy.loginfo(f"Publishing to {motor_name}: {motor_pos}")
        motor_publishers[motor_name].publish(motor_pos)

        rospy.sleep(0.000001)  # Give time for the message to be sent


class MotorHandler:
    @staticmethod
    def get_motor_position(motor_name):
        """Fetch and display the motor position in radians, degrees, and decimal ticks."""
        try:
            # Wait for a single message from the topic with a timeout of 5 seconds
            data = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState, timeout=5.0)

            # Check if the motor name exists and print its position
            if motor_name in data.name:
                index = data.name.index(motor_name)
                position_rad = data.position[index] + math.pi

                # Convert to degrees and decimal encoder ticks
                position_deg = math.degrees(position_rad)

                # Display all three formats
                print(f"Motor: {motor_name}")
                print(f"Position (Radians): {position_rad}")
                print(f"Position (Degrees): {position_deg}")

                return position_deg
            else:
                print(f"No data found for motor named {motor_name}")
                return None

        except rospy.ROSException as e:
            rospy.logerr(f"Timeout or error waiting for joint states: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")
            return None


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

        yA = y + ((self.lf  / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))
        yB = y - ((self.lf  / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))

        zA = z + ((self.lf / 2) * math.sin(math.radians(beta)) * math.cos(math.radians(alpha)))
        zB = z - ((self.lf / 2) * math.sin(math.radians(beta)) * math.cos(math.radians(alpha)))
        print (f"xA = {xA}")
        print (f"xB = {xB}")
        print (f"yA = {yA}")
        print (f"yB = {yB}")
        print (f"zA = {zA}")
        print (f"zB = {zB}")
# Use abs() to take only positive values
        zA = abs(zA)
        zB = abs(zB)
        print (f"zA + = {zA}")
        print (f"zB + = {zB}")
  
        return xA, xB , yA , yB , zA , zB 
    
    #Hip Motor 1DOF
    def InverseKinematicsHip(self, y, z):
        thetaHip = math.atan2(165 + y, z)
        return math.degrees(thetaHip)
    def InverseKinematicsHip2(self, y, z):
        thetaHip = math.atan2(165 - y, z)
        return math.degrees(thetaHip)
    
    
    #Upper&Lower Motor 2DOF
    def InverseKinematics(self, x, z):
        cos_theta2 = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to [-1, 1]
        theta2_radians = math.acos(cos_theta2)
        theta2 = math.degrees(theta2_radians)

        # Calculate theta 1
        theta1_radians = math.atan((self.l2 * math.sin(theta2_radians)) / (self.l1 + self.l2 * math.cos(theta2_radians))) - math.atan2(z , x) + math.pi / 2
        theta1 = math.degrees(theta1_radians)

        # Calculate ct1 for alpha 71
        ct1_squared = self.l6**2 + self.l1**2 - 2 * self.l1 * self.l6 * math.cos(math.pi - theta1_radians)
        ct1 = math.sqrt(ct1_squared)

        # Calculate alpha 71
        cos_alpha71 = (self.l6**2 + ct1**2 - self.l1**2) / (2 * self.l6 * ct1)
        cos_alpha71 = max(-1, min(1, cos_alpha71))  # Clamp to [-1, 1]
        alpha71_radians = math.acos(cos_alpha71)
        alpha71_degrees = math.degrees(alpha71_radians)

        # Calculate At1
        sin_At1 = self.l6 * (math.sin(alpha71_radians) / self.l1)
        sin_At1 = max(-1, min(1, sin_At1))  # Clamp to [-1, 1]
        At1_radians = math.asin(sin_At1)
        At1_degrees = math.degrees(At1_radians)

        # Calculate ct2 for alpha 72
        ct2_squared = self.l3**2 + ct1**2 - 2 * self.l3 * ct1 * math.cos(math.pi - 2.11 + theta2_radians - At1_radians)
        ct2 = math.sqrt(ct2_squared)

        # Calculate alpha 72
        cos_alpha72 = (ct1**2 + ct2**2 - self.l3**2) / (2 * ct1 * ct2)
        cos_alpha72 = max(-1, min(1, cos_alpha72))  # Clamp to [-1, 1]
        alpha72_radians = math.acos(cos_alpha72)
        alpha72_degrees = math.degrees(alpha72_radians)

        # Calculate alpha 73
        cos_alpha73 = (self.l5**2 + ct2**2 - self.l4**2) / (2 * self.l5 * ct2)
        cos_alpha73 = max(-1, min(1, cos_alpha73))  # Clamp to [-1, 1]
        alpha73_radians = math.acos(cos_alpha73)
        alpha73_degrees = math.degrees(alpha73_radians)

        # Calculate theta 3
        theta3 =  (alpha71_degrees + alpha72_degrees + alpha73_degrees)

        return theta1, theta2, theta3

def mainRight():
    # Instantiate the Inverse class
    inv = Inverse()
    
    # Provide values of the right leg
    print("Enter the value of Right legs:")
    xr, zr, yr, alphar, betar, gammar = map(float, input("Enter values for xr, zr, yr, alphar, betar, gammar (separated by spaces): ").split())

    yr = yr-115 #shifted to origin 
    gammar = gammar + 161

    # Call the InverseKinematics method
    xAr , xBr , yAr , yBr , zAr , zBr = inv.InverseKinematicsFoot(xr, yr, zr, alphar, betar)
    theta1r, theta2r, theta3r = inv.InverseKinematics(xBr, zBr)
    
    thetaHipr = inv.InverseKinematicsHip(yBr, zBr)

    theta1l, theta2l, theta3l = inv.InverseKinematics(xAr, zAr)
    thetaHipl = inv.InverseKinematicsHip(yAr, zAr)

    # Print the results right 
    print(f"Theta 1 Lower motor Right: {theta1r} degrees")
    print(f"Theta 2 knee Right: {theta2r} degrees")
    print(f"Theta 3 Upper motor Right: {theta3r} degrees")
    print(f"Theta Hip Upper motor : {thetaHipr} degrees")
    
    theta1shiftedr = theta1r + 180
    theta3shiftedr = theta3r + 27 
    thetaHipshiftedr = thetaHipr + 180 

    print(f"Theta1 shifted Lower motor Right: {theta1shiftedr} degrees")
    print(f"Theta2 shifted knee Right: {theta2r} degrees")
    print(f"Theta3 shifted Upper motor Right: {theta3shiftedr} degrees")
    print(f"Theta Hip shifted : {thetaHipshiftedr} degrees")

    print(f"Theta1 shifted Lower motor Right decimal: {int(theta1shiftedr / 360 * 4096)} ")
    print(f"Theta3 shifted Upper motor Right decimal: {int(theta3shiftedr / 360 * 4096)} ")
    print(f"Theta Hip shifted decimal: {int(thetaHipshiftedr / 360 * 4096)} ")

     # Print the results Left
    print(f"Theta 1 Lower motor Left: {theta1l} degrees")
    print(f"Theta 2 knee Left: {theta2l} degrees")
    print(f"Theta 3 Upper motor Left: {theta3l} degrees")
    print(f"Theta Hip Upper motor : {thetaHipl} degrees")
    
    theta1shiftedl = theta1l + 180
    theta3shiftedl = theta3l + 27 
    thetaHipshiftedl = thetaHipl + 180 

    print(f"Theta1 shifted Lower motor Left: {theta1shiftedl} degrees")
    print(f"Theta2 shifted knee Left: {theta2l} degrees")
    print(f"Theta3 shifted Upper motor Left: {theta3shiftedl} degrees")
    print(f"Theta Hip shifted : {thetaHipshiftedl} degrees")

    print(f"Theta1 shifted Lower motor Left decimal: {int(theta1shiftedl / 360 * 4096)} ")
    print(f"Theta3 shifted Upper motor Left decimal: {int(theta3shiftedl / 360 * 4096)} ")
    print(f"TTheta Hip shifted decimal: {int(thetaHipshiftedl / 360 * 4096)} ")
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl , thetaHipshiftedl, gammar

#second leg (left leg) 
def mainLeft():   
    # Instantiate the Inverse class
    inv = Inverse()
    
    print("Enter the value of Left leg:")
    xl, zl, yl, alphal, betal, gammal = map(float, input("xl, zl, yl, alphal, betal, gammal (separated by spaces): ").split())
    yl = yl+115 #shifted to origin 
    gammal = gammal + 157
    #shift gamma for the left leg
    xAl , xBl , yAl , yBl , zAl , zBl = inv.InverseKinematicsFoot(xl, yl, zl, alphal, betal)
    
    # Call the InverseKinematics method
    theta1r, theta2r, theta3r = inv.InverseKinematics(xAl, zAl)
    
    thetaHipr = inv.InverseKinematicsHip2(yAl, zAl)

    theta1l, theta2l, theta3l = inv.InverseKinematics(xBl, zBl)
    thetaHipl = inv.InverseKinematicsHip2(yBl, zBl)


    # Print the results right 
    print(f"Theta 1 Lower motor Right: {theta1r} degrees")
    print(f"Theta 2 knee Right: {theta2r} degrees")
    print(f"Theta 3 Upper motor Right: {theta3r} degrees")
    print(f"Theta Hip Upper motor : {thetaHipr} degrees")
    
    theta1shiftedr = theta1r + 180
    theta3shiftedr = theta3r + 27 
    thetaHipshiftedr = thetaHipr + 180 

    print(f"Theta1 shifted Lower motor Right: {theta1shiftedr} degrees")
    print(f"Theta2 shifted knee Right: {theta2r} degrees")
    print(f"Theta3 shifted Upper motor Right: {theta3shiftedr} degrees")
    print(f"Theta Hip shifted : {thetaHipshiftedr} degrees")

    print(f"Theta1 shifted Lower motor Right decimal: {int(theta1shiftedr / 360 * 4096)} ")
    print(f"Theta3 shifted Upper motor Right decimal: {int(theta3shiftedr / 360 * 4096)} ")
    print(f"Theta Hip shifted decimal: {int(thetaHipshiftedr / 360 * 4096)} ")

    # Print the results Left
    print(f"Theta 1 Lower motor Left: {theta1l} degrees")
    print(f"Theta 2 knee Left: {theta2l} degrees")
    print(f"Theta 3 Upper motor Left: {theta3l} degrees")
    print(f"Theta Hip Upper motor : {thetaHipl} degrees")
    
    theta1shiftedl = theta1l + 180
    theta3shiftedl = theta3l + 27 
    thetaHipshiftedl = thetaHipl + 180 

    print(f"Theta1 shifted Lower motor Left: {theta1shiftedl} degrees")
    print(f"Theta2 shifted knee Left: {theta2l} degrees")
    print(f"Theta3 shifted Upper motor Left: {theta3shiftedl} degrees")
    print(f"Theta Hip shifted : {thetaHipshiftedl} degrees")

    print(f"Theta1 shifted Lower motor Left decimal: {int(theta1shiftedl / 360 * 4096)} ")
    print(f"Theta3 shifted Upper motor Left decimal: {int(theta3shiftedl / 360 * 4096)} ")
    print(f"Theta Hip shifted decimal: {int(thetaHipshiftedl / 360 * 4096)} ")
    
    
 
    
    

   
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl , thetaHipshiftedl, gammal 

if __name__ == "__main__":
    theta1shiftedrr, theta2rr, theta3shiftedrr, thetaHipshiftedrr, theta1shiftedlr, theta2lr, theta3shiftedlr , thetaHipshiftedlr, gammar= mainRight()
    theta1shiftedrl, theta2rl, theta3shiftedrl, thetaHipshiftedrl, theta1shiftedll, theta2ll, theta3shiftedll , thetaHipshiftedll, gammal = mainLeft()
     # Convert to motor-specific range for the Right leg
    if __name__ == "__main__":
        rospy.init_node('moveMotor', anonymous=True)
       
        theta3shiftedlr = int(theta3shiftedlr / 360 * 4096)
        theta1shiftedlr = int(theta1shiftedlr / 360 * 4096)
        theta3shiftedrr = int(theta3shiftedrr / 360 * 4096)
        theta1shiftedrr = int(theta1shiftedrr / 360 * 4096)
        thetaHipshiftedrr = int(thetaHipshiftedrr / 360 * 4096)
        gammar = int(gammar / 360 * 4096)

         # Convert to motor-specific range for the Left leg
        theta3shiftedll = int(theta3shiftedll / 360 * 4096)
        theta1shiftedll = int(theta1shiftedll / 360 * 4096)
        theta3shiftedrl = int(theta3shiftedrl / 360 * 4096)
        theta1shiftedrl = int(theta1shiftedrl / 360 * 4096)
        thetaHipshiftedrl = int(thetaHipshiftedrl / 360 * 4096) #thetaHipshiftedrl i change it to thetaHipshiftedll
        gammal = int(gammal / 360 * 4096)
    
        print(f"**************** Gamma ***********: {gammar} ")
        steps = 50 
        try:
            currentR11 = int(MotorHandler.get_motor_position('r11') / 360 * 4096)
            currentR12 = int(MotorHandler.get_motor_position('r12') / 360 * 4096)
            currentR13 = int(MotorHandler.get_motor_position('r13') / 360 * 4096)
            currentR14 = int(MotorHandler.get_motor_position('r14') / 360 * 4096)
            currentR15 = int(MotorHandler.get_motor_position('r15') / 360 * 4096)
            currentR16 = int(MotorHandler.get_motor_position('r16') / 360 * 4096)
            
            currentR21 = int(MotorHandler.get_motor_position('r21') / 360 * 4096)
            currentR22 = int(MotorHandler.get_motor_position('r22') / 360 * 4096)
            currentR23 = int(MotorHandler.get_motor_position('r23') / 360 * 4096)
            currentR24 = int(MotorHandler.get_motor_position('r24') / 360 * 4096)
            currentR25 = int(MotorHandler.get_motor_position('r25') / 360 * 4096)
            currentR26 = int(MotorHandler.get_motor_position('r26') / 360 * 4096)

            stepsizeR11 = int((theta3shiftedlr - currentR11) / steps)
            stepsizeR12 = int((theta1shiftedlr - currentR12) / steps)
            stepsizeR13 = int((theta3shiftedrr - currentR13) / steps)
            stepsizeR14 = int((theta1shiftedrr - currentR14) / steps)
            stepsizeR15 = int((thetaHipshiftedrr - currentR15) / steps)
            stepsizeR16 = int((gammar - currentR16) / steps)

            stepsizeR21 = int((theta3shiftedll - currentR21) / steps)
            stepsizeR22 = int((theta1shiftedll - currentR22) / steps)
            stepsizeR23 = int((theta3shiftedrl - currentR23) / steps)
            stepsizeR24 = int((theta1shiftedrl - currentR24) / steps)
            stepsizeR25 = int((thetaHipshiftedrl - currentR25) / steps)
            stepsizeR26 = int((gammal - currentR26) / steps)
            print(f"**************** Current ***********: {currentR16} ")
            print(f"**************** Step size ***********: {stepsizeR16} ")

            newposR11 = currentR11
            newposR12 = currentR12
            newposR13 = currentR13
            newposR14 = currentR14
            newposR15 = currentR15
            newposR16 = currentR16

            newposR21 = currentR21
            newposR22 = currentR22
            newposR23 = currentR23
            newposR24 = currentR24
            newposR25 = currentR25
            newposR26 = currentR26

            for _ in range(steps):
                newposR11 += stepsizeR11
                MoveHandler.move_motor("r11", newposR11)
                newposR12 += stepsizeR12
                MoveHandler.move_motor("r12", newposR12)
                newposR13 += stepsizeR13
                MoveHandler.move_motor("r13", newposR13)
                newposR14 += stepsizeR14
                MoveHandler.move_motor("r14", newposR14)
                newposR15 += stepsizeR15
                MoveHandler.move_motor("r15", newposR15)
                newposR16 += stepsizeR16
                MoveHandler.move_motor("r16", newposR16)
                
                newposR21 += stepsizeR21
                MoveHandler.move_motor("r21", newposR21)
                newposR22 += stepsizeR22
                MoveHandler.move_motor("r22", newposR22)
                newposR23 += stepsizeR23
                MoveHandler.move_motor("r23", newposR23)
                newposR24 += stepsizeR24
                MoveHandler.move_motor("r24", newposR24)
                newposR25 += stepsizeR25
                MoveHandler.move_motor("r25", newposR25)
                newposR26 += stepsizeR26
                MoveHandler.move_motor("r26", newposR26)

                rospy.sleep(0.000001)
                

        except rospy.ROSInterruptException:
            pass

