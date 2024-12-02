#!/usr/bin/env python3
import serial
from robot_driver import driver
import time
# Serial settings for pressure sensor communication
PRESSURE_SERIAL_PORT = '/dev/ttyACM0'
PRESSURE_BAUDRATE = 9600
def get_pressure_data(serial_connection):
    if serial_connection.in_waiting > 0:
        line = serial_connection.readline().decode('utf-8').rstrip()
        try:
            data = line.split(',')
            #RIGHT LEG RR mean R:right l:left side
            front_pressure_Rr = int(data[0])
            front_pressure_Rl = int(data[4])
            back_pressure_Rr = int(data[6])
            back_pressure_Rl = int(data[7])

            front_pressure_Lr = int(data[3])
            front_pressure_Ll = int(data[2])
            back_pressure_Lr = int(data[5])
            back_pressure_Ll = int(data[1])
            print (front_pressure_Rr, front_pressure_Rl, back_pressure_Rr, back_pressure_Rl,
                   front_pressure_Lr,front_pressure_Ll,back_pressure_Lr,back_pressure_Ll )
            return front_pressure_Rr, front_pressure_Rl, back_pressure_Rr, back_pressure_Rl,front_pressure_Lr,front_pressure_Ll,back_pressure_Lr,back_pressure_Ll
        except (IndexError, ValueError):
            print("Error parsing pressure sensor data.")
            return None, None, None, None,None, None, None, None
    return None, None, None, None ,None,None, None, None

def balance_robot(front_pressure_Rr, front_pressure_Rl, back_pressure_Rr, back_pressure_Rl,front_pressure_Lr,front_pressure_Ll,back_pressure_Lr,back_pressure_Ll):
    #Avarage pressures
    front_pressure_right = (front_pressure_Rr + front_pressure_Rl) / 2
    back_pressure_right = (back_pressure_Rr + back_pressure_Rl) / 2
    
    front_pressure_left = (front_pressure_Lr + front_pressure_Ll) /2
    back_pressure_left = (back_pressure_Lr + back_pressure_Ll) /2
    ErrorRight = int(front_pressure_right - back_pressure_right)
    
    ErrorLeft = int(front_pressure_left - back_pressure_left)
    factor = 0.009
    
    if abs(ErrorRight) > 50:
        angleR = ErrorRight * factor
        myRobot.set_motor_position(16, (myRobot.get_motor_position(16) - angleR))
    if abs(ErrorLeft) > 50:
        angleL = ErrorLeft * factor
        myRobot.set_motor_position(26, (myRobot.get_motor_position(26) - angleL))
    
if __name__ == "__main__":
    try:
        # Initialize pressure sensor serial connection
        pressure_serial = serial.Serial(PRESSURE_SERIAL_PORT, PRESSURE_BAUDRATE, timeout=2)
        pressure_serial.reset_input_buffer()
        
        myRobot = driver()
        myRobot.set_all_motors([])
        while 1:
            # Read pressure data from pressure serial
            FRr, FRl, BRr, BRl,FLr,FLl,BLr,BLl = get_pressure_data(pressure_serial)
            if None not in (FRr, FRl, BRr, BRl,FLr,FLl,BLr,BLl):
                # Directly call balance_robot without reopening motor_serial
                balance_robot(FRr, FRl, BRr, BRl,FLr,FLl,BLr,BLl)
            
    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if pressure_serial.is_open:
            pressure_serial.close()
