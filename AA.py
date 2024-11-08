
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
    #shift gamma for the left leg
    
    
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
    print(f"TTheta Hip shifted decimal: {int(thetaHipshiftedl / 360 * 4096)} ")
    
   
    return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl , thetaHipshiftedl, gammal

def moveMotor():
    rospy.init_node('moveMotor', anonymous=True)
    
    #For the Right leg
    pubTh1r14 = rospy.Publisher('r14', Int16, queue_size=10)
    pubTh3r13 = rospy.Publisher('r13', Int16, queue_size=10)
    pubThHipr15 = rospy.Publisher('r15', Int16, queue_size=10)
    pubTh1r12 = rospy.Publisher('r12', Int16, queue_size=10)
    pubTh3r11 = rospy.Publisher('r11', Int16, queue_size=10)
    pubGammarr16 = rospy.Publisher('r16', Int16, queue_size=10)

    #For the Left leg
    pubTh1r24 = rospy.Publisher('r24', Int16, queue_size=10)
    pubTh3r23 = rospy.Publisher('r23', Int16, queue_size=10)
    pubThHipr25 = rospy.Publisher('r25', Int16, queue_size=10)
    pubTh1r22 = rospy.Publisher('r22', Int16, queue_size=10)
    pubTh3r21 = rospy.Publisher('r21', Int16, queue_size=10)
    pubGammalr26 = rospy.Publisher('r26', Int16, queue_size=10)

    theta1shiftedrr, theta2rr, theta3shiftedrr, thetaHipshiftedrr, theta1shiftedlr, theta2lr, theta3shiftedlr , thetaHipshiftedlr, gammar= mainRight()
    theta1shiftedrl, theta2rl, theta3shiftedrl, thetaHipshiftedrl, theta1shiftedll, theta2ll, theta3shiftedll , thetaHipshiftedll, gammal= mainLeft()



    
    rospy.sleep(1)  # Allow time for publisher to connect
    
    # Log info for the Right leg
    rospy.loginfo(f"Publishing to r14: {mov14}")
    rospy.loginfo(f"Publishing to r13: {mov13}")
    rospy.loginfo(f"Publishing to r15: {mov15}")
    rospy.loginfo(f"Publishing to r12: {mov12}")
    rospy.loginfo(f"Publishing to r11: {mov11}")
    rospy.loginfo(f"Publishing to r16: {mov16}")

    # Log info for the Left leg
    rospy.loginfo(f"Publishing to r24: {mov24}")
    rospy.loginfo(f"Publishing to r23: {mov23}")
    rospy.loginfo(f"Publishing to r25: {mov25}")
    rospy.loginfo(f"Publishing to r22: {mov22}")
    rospy.loginfo(f"Publishing to r21: {mov21}")
    rospy.loginfo(f"Publishing to r26: {mov26}")

    # Publish for the Right leg
    pubTh1r14.publish(mov14)
    pubTh3r13.publish(mov13)
    pubThHipr15.publish(mov15)
    pubTh1r12.publish(mov12)
    pubTh3r11.publish(mov11)
    pubGammarr16.publish(mov16)

    # Publish for the Left leg
    pubTh1r24.publish(mov24)
    pubTh3r23.publish(mov23)
    pubThHipr25.publish(mov25)
    pubTh1r22.publish(mov22)
    pubTh3r21.publish(mov21)
    pubGammalr26.publish(mov26)
    
    # Example: Assuming the motor has a 12-bit encoder with 4096 ticks per revolution
    MAX_ENCODER_VALUE = 4096  # Adjust if your motor has a different resolution

    def rad_to_deg(radians):
        #"""Convert radians to degrees."""
        return radians * (180 / math.pi)

    def rad_to_decimal(radians):
        #"""Convert radians to encoder ticks (decimal position)."""
        return (radians / (2 * math.pi)) * MAX_ENCODER_VALUE

    def get_motor_position(motor_name):
        """Fetch and display the motor position in radians, degrees, and decimal ticks."""
       # Wait for a single message from the topic
        data = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState)

      # Check if the motor name exists and print its position
        if motor_name in data.name:
            index = data.name.index(motor_name)
            position_rad = data.position[index]

            # Convert to degrees and decimal encoder ticks
            position_deg = rad_to_deg(position_rad)
            position_decimal = rad_to_decimal(position_rad)

            # Display all three formats
            print(f"Motor: {motor_name}")
            print(f"Position (Radians): {position_rad:.6f}")
            print(f"Position (Degrees): {position_deg:.2f}")
            print(f"Position (Decimal): {position_decimal:.2f}")
            return position_deg + 180
        else:
            print(f"No data found for motor named {motor_name}")

    # Convert to motor-specific range for the Right leg
    dectheta3shiftedlr = int(theta3shiftedlr / 360 * 4096)
    dectheta1shiftedlr = int(theta1shiftedlr / 360 * 4096)
    dectheta3shiftedrr = int(theta3shiftedrr / 360 * 4096)
    dectheta1shiftedrr = int(theta1shiftedrr / 360 * 4096)
    decthetaHipshiftedrr = int(thetaHipshiftedrr / 360 * 4096)
    decgammar = int(gammar / 360 * 4096)

    # Convert to motor-specific range for the Left leg
    dectheta3shiftedll = int(theta3shiftedll / 360 * 4096)
    dectheta1shiftedll = int(theta1shiftedll / 360 * 4096)
    dectheta3shiftedrl = int(theta3shiftedrl / 360 * 4096)
    dectheta1shiftedrl = int(theta1shiftedrl / 360 * 4096)
    decthetaHipshiftedrl = int(thetaHipshiftedrl / 360 * 4096) #thetaHipshiftedrl i change it to thetaHipshiftedll
    decgammal = int(gammal / 360 * 4096)

    # All current in decimal
    current_position_r14 = get_motor_position('r14') / 360 * 4096
    current_position_r13 = get_motor_position('r13') / 360 * 4096
    current_position_r15 = get_motor_position('r15') / 360 * 4096
    current_position_r12 = get_motor_position('r12') / 360 * 4096
    current_position_r11 = get_motor_position('r11') / 360 * 4096
    current_position_r16 = get_motor_position('r16') / 360 * 4096
    current_position_r24 = get_motor_position('r24') / 360 * 4096
    current_position_r23 = get_motor_position('r23') / 360 * 4096
    current_position_r25 = get_motor_position('r25') / 360 * 4096
    current_position_r22 = get_motor_position('r22') / 360 * 4096
    current_position_r21 = get_motor_position('r21') / 360 * 4096
    current_position_r26 = get_motor_position('r26') / 360 * 4096

    mov11 = current_position_r11 + 1
    mov12 = current_position_r12 + 1
    mov13 = current_position_r13 + 1
    mov14 = current_position_r14 + 1
    mov15 = current_position_r15 + 1
    mov16 = current_position_r16 + 1
    mov21 = current_position_r21 + 1
    mov22 = current_position_r22 + 1
    mov23 = current_position_r23 + 1
    mov24 = current_position_r24 + 1
    mov25 = current_position_r25 + 1
    mov26 = current_position_r26 + 1

    rospy.sleep(1)  # Give time for messages to be sent

        # Main loop
    while (mov11 != dectheta3shiftedlr or mov12 != dectheta1shiftedlr or mov13 != dectheta3shiftedrr or mov14 != dectheta1shiftedrr or mov15 != decthetaHipshiftedrr or mov16 != decgammar or mov21 != dectheta3shiftedll or mov22 != dectheta1shiftedll or mov23 != dectheta3shiftedrl or mov24 != dectheta1shiftedrl or mov25 != decthetaHipshiftedrl or mov26 != decgammal):

        moveMotor()

        # Increment variables as needed
        if mov11 != dectheta3shiftedlr:
            mov11 += 1
        if mov12 != dectheta1shiftedlr:
            mov12 += 1
        if mov13 != dectheta3shiftedrr:
            mov13 += 1
        if mov14 != dectheta1shiftedrr:
            mov14 += 1
        if mov15 != decthetaHipshiftedrr:
            mov15 += 1
        if mov16 != decgammar:
            mov16 += 1
        if mov21 != dectheta3shiftedll:
            mov21 += 1
        if mov22 != dectheta1shiftedll:
            mov22 += 1
        if mov23 != dectheta3shiftedrl:
            mov23 += 1
        if mov24 != dectheta1shiftedrl:
            mov24 += 1
        if mov25 != decthetaHipshiftedrl:
            mov25 += 1
        if mov26 != decgammal:
            mov26 += 1


if __name__ == "__main__":
    try:
        moveMotor()
    except rospy.ROSInterruptException:
        pass
