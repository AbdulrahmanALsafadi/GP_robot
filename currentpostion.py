from pypot.dynamixel.io import DxlIO

def get_motor_positions(port='/dev/ttyUSB0'):
    with DxlIO(port) as dxl_io:
        # Scan for motor IDs
        motor_IDs = dxl_io.scan()

        # Get the current positions of the motors
        motor_positions = dxl_io.get_present_position(motor_IDs)

        # Create a dictionary to save motor IDs and their positions
        motor_data = {motor_ID: motor_positions[i] for i, motor_ID in enumerate(motor_IDs)}

    # Return the saved motor data (ID and position)
    return motor_data

# Example of how to call the function
motor_data = get_motor_positions()
print(motor_data)
