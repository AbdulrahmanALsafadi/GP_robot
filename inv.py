import math

class Inverse:
    l1 = 203
    l2 = 160.84
    l3 = 100       #60
    l4 = 324
    l5 = 58
    l6 = 95.6
    lf = 98.09

    def InverseKinematicsFoot(self, x, y, z, alpha, beta, gamma):
        x1 = x + ((self.lf / 2) * math.cos(math.radians(alpha)) * math.sin(math.radians(beta)))
        x2 = x - ((self.lf / 2) * math.cos(math.radians(alpha)) * math.sin(math.radians(beta)))

        y1 = y + ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))
        y2 = y - ((self.lf / 2) * math.cos(math.radians(beta)) * math.cos(math.radians(alpha)))

        z1 = z + ((self.lf / 2) * math.sin(math.radians(alpha)) * math.cos(math.radians(beta)))
        z2 = z - ((self.lf / 2) * math.sin(math.radians(alpha)) * math.cos(math.radians(beta)))

        return x1, x2, y1, y2, z1, z2

    def InverseKinematicsHip(self, y, z):
        theta = math.atan2(z, y)
        return math.degrees(theta)

    def InverseKinematics(self, y, z):
        theta2_radians = math.acos((y * 2 + z * 2 - self.l1 * 2 - self.l2 * 2) / (2 * self.l1 * self.l2))
        theta2_degrees = math.degrees(theta2_radians)

        theta1_radians = math.atan((self.l2 * math.sin(theta2_radians)) / (self.l1 + self.l2 * math.cos(theta2_radians))) - math.atan(z / y) + math.pi / 2
        theta1_degrees = math.degrees(theta1_radians)

        ct1_squared = self.l6 * 2 + self.l1 * 2 - 2 * self.l1 * self.l6 * math.cos(math.pi - theta1_radians)
        ct1 = math.sqrt(ct1_squared)

        alpha71_radians = math.acos((self.l6 * 2 + ct1 * 2 - self.l1 ** 2) / (2 * self.l6 * ct1))
        alpha71_degrees = math.degrees(alpha71_radians)

        At1_radians = math.asin(self.l6 * (math.sin(alpha71_radians) / self.l1))
        At1_degrees = math.degrees(At1_radians)

        ct2_squared = self.l3 * 2 + ct1 * 2 - 2 * self.l3 * ct1 * math.cos(math.pi - 2.11 + theta2_radians - At1_radians)
        ct2 = math.sqrt(ct2_squared)

        alpha72_radians = math.acos((ct1 * 2 + ct2 * 2 - self.l3 ** 2) / (2 * ct1 * ct2))
        alpha72_degrees = math.degrees(alpha72_radians)

        alpha73_radians = math.acos((self.l5 * 2 + ct2 * 2 - self.l4 ** 2) / (2 * self.l5 * ct2))
        alpha73_degrees = math.degrees(alpha73_radians)

        theta3_degrees = 180 - (alpha71_degrees + alpha72_degrees + alpha73_degrees)

        return theta1_degrees, theta2_degrees, theta3_degrees

# Main function to test the class
def main():
    inv = Inverse()

    # user enter values for x, y, z, alpha, beta, gamma
    user_input_RightLeg = input("Enter integer values for x, y, z, alpha, beta, gamma  for the right leg separated by spaces: ")
    user_input_LeftLeg = input("Enter integer values for x, y, z, alpha, beta, gamma  for the left leg separated by spaces: ")
    print()


    # map the user inpute to the variables for the right leg
    xr, yr, zr, alphar, betar, gammar = map(float, user_input_RightLeg.split())

    # Calculate and print the results
    foot_results = inv.InverseKinematicsFoot(xr, yr, zr, alphar, betar, gammar)

    # Print results from InverseKinematicsFoot
    print("InverseKinematicsFoot Results: Right side position")
    print(f"x1: {foot_results[0]}")
    print(f"y1: {foot_results[2]}")
    print(f"z1: {foot_results[4]}")

    print()

    print("InverseKinematicsFoot Results: left side position")
    print(f"x2: {foot_results[1]}")
    print(f"y2: {foot_results[3]}")
    print(f"z2: {foot_results[5]}")

    print()

    # Split the result to right half and left half for the foot posision
    x1, x2, y1, y2, z1, z2 = foot_results

    # Use y1 , z1 for the right side and z2 , y2 for the left side of the leg
    hip_result = inv.InverseKinematicsHip(y1, z1)
    print(f"InverseKinematicsHip result (Hip-theta): {hip_result} degrees")

    print()

    kinematics_results = inv.InverseKinematics(y1, z1)
    print("InverseKinematics results for right side of the right leg :")
    print(f"theta1 (Lower Motor): {kinematics_results[0]:.2f} degrees")
    print(f"theta2 (Knee): {kinematics_results[1]:.2f} degrees")
    print(f"theta3 (Upper motor): {kinematics_results[2]:.2f} degrees")

    print()

    print("InverseKinematics results for left side of the right leg :")
    kinematics_results = inv.InverseKinematics(y2, z2)
    print(f"theta1 (Lower Motor): {kinematics_results[0]:.2f} degrees")
    print(f"theta2 (Knee): {kinematics_results[1]:.2f} degrees")
    print(f"theta3 (Upper motor): {kinematics_results[2]:.2f} degrees")

    print()
    print()

    # map the user inpute to the variables for the left leg
    xl, yl, zl, alphal, betal, gammal = map(float, user_input_LeftLeg.split())

    # Calculate and print the results
    foot_results = inv.InverseKinematicsFoot(xl, yl, zl, alphal, betal, gammal)

    # Print results from InverseKinematicsFoot
    print("InverseKinematicsFoot Results: Right side position")
    print(f"x1: {foot_results[0]}")
    print(f"y1: {foot_results[2]}")
    print(f"z1: {foot_results[4]}")

    print()

    print("InverseKinematicsFoot Results: left side position")
    print(f"x2: {foot_results[1]}")
    print(f"y2: {foot_results[3]}")
    print(f"z2: {foot_results[5]}")

    print()

    # Split the result to right half and left half for the foot posision
    x1, x2, y1, y2, z1, z2 = foot_results

    # Use y1 , z1 for the right side and z2 , y2 for the left side of the leg
    hip_result = inv.InverseKinematicsHip(y1, z1)
    print(f"InverseKinematicsHip result (Hip-theta): {hip_result} degrees")

    print()

    kinematics_results = inv.InverseKinematics(y1, z1)
    print("InverseKinematics results for right side of the left leg :")
    print(f"theta1 (Lower Motor): {kinematics_results[0]:.2f} degrees")
    print(f"theta2 (Knee): {kinematics_results[1]:.2f} degrees")
    print(f"theta3 (Upper motor): {kinematics_results[2]:.2f} degrees")
    
    print()

    print("InverseKinematics results for left side of the left leg :")
    kinematics_results = inv.InverseKinematics(y2, z2)
    print(f"theta1 (Lower Motor): {kinematics_results[0]:.2f} degrees")
    print(f"theta2 (Knee): {kinematics_results[1]:.2f} degrees")
    print(f"theta3 (Upper motor): {kinematics_results[2]:.2f} degrees")


if _name_ == "_main_":
    main()
