#!/usr/bin/env python3
import serial
import time
import math
import rospy
from sensor_msgs.msg import JointState

class MotorHandler:
    @staticmethod
    def get_motor_position(motor_name):
        try:
            data = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState, timeout=5.0)
            if motor_name in data.name:
                index = data.name.index(motor_name)
                position_rad = data.position[index] + math.pi
                position_deg = math.degrees(position_rad)
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

# Serial settings for motor communication
MOTOR_SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT891LRN-if00-port0'
MOTOR_BAUDRATE = 1000000

# Serial settings for pressure sensor communication
PRESSURE_SERIAL_PORT = '/dev/ttyACM0'
PRESSURE_BAUDRATE = 9600

DXL_GOAL_POSITION = 0x1E
DXL_PRESENT_POSITION = 0x24
DXL_INSTRUCTION_WRITE = 0x03
DXL_INSTRUCTION_READ = 0x02

# Motor configurations
motors_config = {
    16: {"CW_ANGLE_LIMIT": 0, "CCW_ANGLE_LIMIT": 4095},
    26: {"CW_ANGLE_LIMIT": 0, "CCW_ANGLE_LIMIT": 4095}
}

def calculate_checksum(packet):
    return (~sum(packet) & 0xFF)

def send_packet(serial_connection, packet):
    serial_connection.write(bytearray(packet))

def build_write_packet(motor_id, address, value):
    packet = [0xFF, 0xFF, motor_id, 0x05, DXL_INSTRUCTION_WRITE, address, value & 0xFF, (value >> 8) & 0xFF]
    packet.append(calculate_checksum(packet[2:]))
    return packet

def move_motor_to_position(serial_connection, motor_id, target_position):
    if motor_id in motors_config:
        CW_ANGLE_LIMIT = motors_config[motor_id]["CW_ANGLE_LIMIT"]
        CCW_ANGLE_LIMIT = motors_config[motor_id]["CCW_ANGLE_LIMIT"]
    else:
        #print(f"Motor ID {motor_id} not found in configuration.")
        return

    if target_position < CW_ANGLE_LIMIT:
        #print(f"Target position {target_position} is below the safe range. Moving to minimum limit {CW_ANGLE_LIMIT}.")
        target_position = CW_ANGLE_LIMIT
    elif target_position > CCW_ANGLE_LIMIT:
       #print(f"Target position {target_position} exceeds the safe range. Moving to maximum limit {CCW_ANGLE_LIMIT}.")
        target_position = CCW_ANGLE_LIMIT

    #print(f"Moving motor ID {motor_id} to position {target_position}")
    packet = build_write_packet(motor_id, DXL_GOAL_POSITION, target_position)
    send_packet(serial_connection, packet)
    #print(f"Motor ID {motor_id} set to position {target_position}")

def get_pressure_data(serial_connection):
    if serial_connection.in_waiting > 0:
        line = serial_connection.readline().decode('utf-8').rstrip()
        try:
            data = line.split(',')
            front_pressure_A0 = int(data[0])
            back_pressure_A1 = int(data[1])
            front_pressure_A2 = int(data[2])
            back_pressure_A3 = int(data[3])
            print (front_pressure_A0, back_pressure_A1, front_pressure_A2, back_pressure_A3)
            return front_pressure_A0, back_pressure_A1, front_pressure_A2, back_pressure_A3
        except (IndexError, ValueError):
            print("Error parsing pressure sensor data.")
            return None, None, None, None
    return None, None, None, None

def balance_robot(motor_serial, front_pressure_right, back_pressure_right, front_pressure_left, back_pressure_left):
    ErrorRight = int(front_pressure_right - back_pressure_right)
    ErrorLeft = int(front_pressure_left - back_pressure_left)
    factor = 0.05

    if abs(ErrorRight) > 100:
        angleR = ErrorRight * factor
        currentR16 = int(MotorHandler.get_motor_position('r16') / 360 * 4096)
        correctionR = int(currentR16 - angleR)
        move_motor_to_position(motor_serial, 16, correctionR)

    if abs(ErrorLeft) > 100:
        angleL = ErrorLeft * factor
        currentR26 = int(MotorHandler.get_motor_position('r26') / 360 * 4096)
        correctionL = int(currentR26 - angleL)
        move_motor_to_position(motor_serial, 26, correctionL)

    
if __name__ == "__main__":
    rospy.init_node('balance_robot', anonymous=True)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')

    try:
        # Initialize motor serial connection
        motor_serial = serial.Serial(MOTOR_SERIAL_PORT, MOTOR_BAUDRATE, timeout=1)
        # Initialize pressure sensor serial connection
        pressure_serial = serial.Serial(PRESSURE_SERIAL_PORT, PRESSURE_BAUDRATE, timeout=1)

        pressure_serial.reset_input_buffer()
        
        while not rospy.is_shutdown():
            # Read pressure data from pressure serial
            front_pressure_A0, back_pressure_A1, front_pressure_A2, back_pressure_A3 = get_pressure_data(pressure_serial)
            if None not in (front_pressure_A0, back_pressure_A1, front_pressure_A2, back_pressure_A3):
                balance_robot(motor_serial, front_pressure_A0, back_pressure_A1, front_pressure_A2, back_pressure_A3)
            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if motor_serial.is_open:
            motor_serial.close()
        if pressure_serial.is_open:
            pressure_serial.close()
