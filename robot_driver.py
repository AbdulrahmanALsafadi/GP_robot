import pypot.robot
import time
import numpy as np



class driver:
    def __init__(self):
        self.robot = pypot.robot.from_json("config.json")

        self.right_positions = [-56.75, 40.84, -55.78, 40.66, 0.57, 59.82]
        self.left_positions = [-63.08, 42.77, -55.78, 41.19, 2.86, 57.63]    
        time.sleep(0.2)

        
    def get_right_position(self, id):
        return self.robot.motors[id - 11].present_position

    def get_left_position(self, id):
        return self.robot.motors[id - 15].present_position
    
    def get_motor_position(self, id):
        if id < 17:
            return self.robot.motors[id - 11].present_position 
        else:
            return self.robot.motors[id - 15].present_position

    def set_motor_position(self, id, goal):
        if id < 17:
            self.robot.motors[id - 11].compliant = False
            self.robot.motors[id - 11].goal_position = goal
        else:
            self.robot.motors[id - 15].compliant = False
            self.robot.motors[id - 15].goal_position = goal

    def get_all_positions(self):
        all_pos = []
        for m in self.robot.motors: 
            all_pos.append(m.present_position)
        return all_pos

    def release_right(self):
        for m in self.robot.right:
            m.compliant = True
            time.sleep(0.1)

    def release_left(self):
        for m in self.robot.left:
            m.compliant = True
            time.sleep(0.1)
    
    def go_all_motors(self, goal, steps):
        current = self.get_all_positions()

        seq = np.linspace(current, goal, steps)
        for i in range(len(seq)):
            self.set_all_motors(seq[i])
            time.sleep(0.0015)#for one leg = 0,015 the leakage 

    def set_all_motors(self, cmd):
        #ids = [11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26]
        for i in range(len(cmd)):
            self.robot.motors[i].compliant = False
            self.robot.motors[i].goal_position = cmd[i]


    def stand(self):
        self.right_positions = [116.72, 199.25, 166.72, 198.19, 185.63, 203.99]
        self.left_positions = [108.72, 201.80, 115.14, 199.34, 182.29, 204.96]



  