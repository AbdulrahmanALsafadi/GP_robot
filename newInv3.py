#Best Code 

import math
import time
from std_msgs.msg import Int16
import rospy
from sensor_msgs.msg import JointState
from robot_driver import driver
        

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
        theta3 =  180 - (alpha71_degrees + alpha72_degrees + alpha73_degrees)

        return theta1, theta2, theta3

    def mainRight(xr, zr, yr, alphar, betar, gammar):
        # Instantiate the Inverse class
        inv = Inverse()

        yr = yr-115
        
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
        
        theta1shiftedr = theta1r 
        theta3shiftedr = theta3r  
        thetaHipshiftedr = thetaHipr  

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
        
        theta1shiftedl = theta1l 
        theta3shiftedl = theta3l
        thetaHipshiftedl = thetaHipl  

        print(f"Theta1 shifted Lower motor Left: {theta1shiftedl} degrees")
        print(f"Theta2 shifted knee Left: {theta2l} degrees")
        print(f"Theta3 shifted Upper motor Left: {theta3shiftedl} degrees")
        print(f"Theta Hip shifted : {thetaHipshiftedl} degrees")

        print(f"Theta1 shifted Lower motor Left decimal: {int(theta1shiftedl / 360 * 4096)} ")
        print(f"Theta3 shifted Upper motor Left decimal: {int(theta3shiftedl / 360 * 4096)} ")
        print(f"TTheta Hip shifted decimal: {int(thetaHipshiftedl / 360 * 4096)} ")
        return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl , thetaHipshiftedl, gammar

    #second leg (left leg) 
    def mainLeft(xl, zl, yl, alphal, betal, gammal):   
        # Instantiate the Inverse class
        inv = Inverse()
        
        yl = yl + 115
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
        
        theta1shiftedr = theta1r 
        theta3shiftedr = theta3r 
        thetaHipshiftedr = thetaHipr 

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
        
        theta1shiftedl = theta1l 
        theta3shiftedl = theta3l  
        thetaHipshiftedl = thetaHipl 

        print(f"Theta1 shifted Lower motor Left: {theta1shiftedl} degrees")
        print(f"Theta2 shifted knee Left: {theta2l} degrees")
        print(f"Theta3 shifted Upper motor Left: {theta3shiftedl} degrees")
        print(f"Theta Hip shifted : {thetaHipshiftedl} degrees")

        print(f"Theta1 shifted Lower motor Left decimal: {int(theta1shiftedl / 360 * 4096)} ")
        print(f"Theta3 shifted Upper motor Left decimal: {int(theta3shiftedl / 360 * 4096)} ")
        print(f"Theta Hip shifted decimal: {int(thetaHipshiftedl / 360 * 4096)} ")
        
        
    
        
        

    
        return theta1shiftedr, theta2r, theta3shiftedr, thetaHipshiftedr, theta1shiftedl, theta2l, theta3shiftedl , thetaHipshiftedl, gammal 



if __name__ == "__main__":
    myRobot = driver()
    def parse_input(input_str):
        """
        Parse the user input and fill missing values with 0.
        If fewer than 6 values are provided, fill the remaining ones with 0s.
        """
        values = list(map(float, input_str.split()))
        # Ensure there are exactly 6 values by appending zeros if needed
        while len(values) < 6:
            values.append(0)
        return values

    same_values = int(input("Do you want the same values for both legs? (1 for Yes, 0 for No): ").strip())

    if same_values == 1:
        print("Enter the values for both legs (x, z, y, alpha, beta, gamma]):")
        values = parse_input(input("x, z, y, alpha, beta, gamma separated by spaces]: "))
        xr, zr, yr, alphar, betar, gammar = values
        xl, zl, yl, alphal, betal, gammal = values
    else:
        print("Enter the value of Right leg:")
        xr, zr, yr, alphar, betar, gammar = parse_input(
            input("x, z, y, alpha, beta, gamma separated by spaces]: ")
        )
        print("Enter the value of Left leg:")
        xl, zl, yl, alphal, betal, gammal = parse_input(
            input("x, z, y, alpha, beta, gamma separated by spaces]: ")
        )

    r14, _, r13, r15, r12, _, r11, _, r16 = Inverse.mainRight(xr, zr, yr, alphar, betar, gammar)
    l24, _, l23, l25, l22, _, l21, _, l26 = Inverse.mainLeft(xl, zl, yl, alphal, betal, gammal)


    myRobot.go_all_motors([r11, r12, r13, r14, r15, r16, l21, l22, l23, l24, l25, l26], 85)
    print(myRobot.get_all_positions())

   # set-down[63.71, 55.08, 70.57, 55.78, -2.77, 58.77, 71.19, 55.78, 58.879999999999995, 57.98, 6.9, 57.36]
   #stable set [-51.099999999999994, 99.38, -42.129999999999995, 98.24, -4.35, 59.21, -39.93000000000001, 98.86, -60.33, 100.79, 6.11, 57.89]
   #STABLE SET [-63.849999999999994, 101.14, -49.78, 99.3, -2.95, 59.03, -49.959999999999994, 100.44, -60.42, 100.97, 8.84, 57.63]
 
#Stand up1 better [89.12, 19.12, 100.64, 17.01, -3.12, 25.45, 93.34, 22.9, 83.67, 22.99, 3.38, 20.53]
#Stand up2 [77.96000000000001, 31.78, 88.07, 30.9, 1.27, 18.95, 81.91, 34.68, 72.15, 34.51, 6.11, 13.85]

#take step [69.69, 30.55, 73.38, 34.15, 0.48, 25.19, 4.02, 64.31, 3.4899999999999984, 63.16, 5.93, -9.1]
#second 69.69, 30.55, 73.38, 34.15, -5.93, 25.19,69.69, 30.55, 73.38, 34.15, 0.48, 25.19