#last  work
import tkinter as tk
from robot_driver import driver
from InvG import Inverse
import time


def move_motor(id, value):
    """
    Move a specific motor to the desired position.

    Args:
        id (int): Motor ID to control.
        value (float): Target position.
    """
    myRobot.set_motor_position(id, float(value))

def move_robot():
    global xr, zr, yr, alphaR, betaR, gammaR, xl, zl, yl, alphaL, betaL, gammaL
    
    r14, _, r13, r15, r12, _, r11, _, r16 = Inverse.mainRight(xr, zr, yr, alphaR, betaR, gammaR)
    l24, _, l23, l25, l22, _, l21, _, l26 = Inverse.mainLeft(xl, zl, yl, alphaL, betaL, gammaL)
    myRobot.set_all_motors([r11, r12, r13, r14, r15, r16, l21, l22, l23, l24, l25, l26])

def move_robot_to(xr, zr, yr, alphaR, betaR, gammaR, xl, zl, yl, alphaL, betaL, gammaL, step):    
    r14, _, r13, r15, r12, _, r11, _, r16 = Inverse.mainRight(xr, zr, yr, alphaR, betaR, gammaR)
    l24, _, l23, l25, l22, _, l21, _, l26 = Inverse.mainLeft(xl, zl, yl, alphaL, betaL, gammaL)
    myRobot.go_all_motors([r11, r12, r13, r14, r15, r16, l21, l22, l23, l24, l25, l26], step)

def ry_slider_cb(value):
    global yr
    yr = int(value)
    move_robot()

def ly_slider_cb(value):
    global yl
    yl = int(value)
    move_robot()

def rx_slider_cb(value):
    global xr
    xr = int(value)
    move_robot()

def lx_slider_cb(value):
    global xl
    xl = int(value)
    move_robot()

#Z
def rz_slider_cb(value):
    global zr
    zr = int(value)
    move_robot()

def lz_slider_cb(value):
    global zl
    zl = int(value)
    move_robot()

#Alpha
def lalpha_slider_cb(value):
    global alphaL
    alphaL = int(value)
    move_robot()

def ralpha_slider_cb(value):
    global alphaR
    alphaR = int(value)
    move_robot()

#Beta
def lbeta_slider_cb(value):
    global betaL
    betaL = int(value)
    move_robot()

def rbeta_slider_cb(value):
    global betaR
    betaR = int(value)
    move_robot()

#Gamma
def lgamma_slider_cb(value):
    global gammaL
    gammaL = int(value)
    move_robot()

def rgamma_slider_cb(value):
    global gammaR
    gammaR = int(value)
    move_robot()


def stand_up():
   
    time.sleep(0.2)
    v = [0, 350, 0, 0, 0, -15    , 0, 350, 0, 0, 0, -15     , 25]
    move_robot_to(  v[0], v[1], v[2], v[3], v[4], v[5]    , v[6], v[7], v[8], v[9], v[10], v[11]     , 10)
    rx_slider.set(v[0])
    rz_slider.set(v[1])
    ry_slider.set(v[2])
    ralpha_slider.set(v[3])
    rbeta_slider.set(v[4])
    rgamma_slider.set(v[5])

    lx_slider.set(v[6])
    lz_slider.set(v[7])
    ly_slider.set(v[8])
    lalpha_slider.set(v[9])
    lbeta_slider.set(v[10])
    lgamma_slider.set(v[11])

def do_seq():

    
    move_robot_to(  0, 350, -15, 0, 0, -15    , 0, 350, 30, 0, 0, -15     , 25)    
    
    time.sleep(2)

    #move_robot_to(50, 320, -15, 18, 0, -13    , -50, 350, 30, 12, 0, -12, 25)
    
    v = [50, 320, -15, 18, 0, -13    , -50, 350, 30, 12, 0, -12]
    move_robot_to(  v[0], v[1], v[2], v[3], v[4], v[5]    , v[6], v[7], v[8], v[9], v[10], v[11]     , 20) #280 to 310   

    rx_slider.set(v[0])
    rz_slider.set(v[1])
    ry_slider.set(v[2])
    ralpha_slider.set(v[3])
    rbeta_slider.set(v[4])
    rgamma_slider.set(v[5])

    lx_slider.set(v[6])
    lz_slider.set(v[7])
    ly_slider.set(v[8])
    lalpha_slider.set(v[9])
    lbeta_slider.set(v[10])
    lgamma_slider.set(v[11])

def do_seq2():

    
    move_robot_to(50, 350, -15, 18, 0, -12    , -50, 350, 30, 12, 0, -16, 25)    
    
    #time.sleep(2)

    move_robot_to( 0, 350, 0, 18, 0, -12    , 0, 350, 0, 12, 0, -16, 2     , 20)
    
    v = [-50, 350, 30, 18, 0, -13    , 50, 320, -15, 12, 0, -12]
    move_robot_to(  v[0], v[1], v[2], v[3], v[4], v[5]    , v[6], v[7], v[8], v[9], v[10], v[11]     , 20)
    
    rx_slider.set(v[0])
    rz_slider.set(v[1])
    ry_slider.set(v[2])
    ralpha_slider.set(v[3])
    rbeta_slider.set(v[4])
    rgamma_slider.set(v[5])

    lx_slider.set(v[6])
    lz_slider.set(v[7])
    ly_slider.set(v[8])
    lalpha_slider.set(v[9])
    lbeta_slider.set(v[10])
    lgamma_slider.set(v[11])



if __name__ == "__main__":

    # Initialize the robot driver
    myRobot = driver()

    # Step size for increasing or decreasing motor positions
    STEP_SIZE = 5  # Adjust as needed

    # Create the main window
    root = tk.Tk()
    root.title("Humanoid Robot Motor Control")

    # Create frames for left and right leg controls
    right_frame = tk.Frame(root)
    right_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

    left_frame = tk.Frame(root)
    left_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

    # Buttons frame (bottom)
    button_frame = tk.Frame(root)
    button_frame.grid(row=1, column=0, columnspan=2, pady=10)

    # Right Leg Sliders (Place them in the right_frame)
    slider_label_hip_15 = tk.Label(right_frame, text="(Hip - Right)")
    slider_label_hip_15.pack()

    ry_slider = tk.Scale(right_frame, from_=-200, to=200, orient=tk.HORIZONTAL, length=300, command=ry_slider_cb)
    ry_slider.set(0)
    ry_slider.pack()

    # For X Right
    slider_label_hip_xr = tk.Label(right_frame, text="X axis (Right)")
    slider_label_hip_xr.pack()

    rx_slider = tk.Scale(right_frame, from_=-200, to=200, orient=tk.HORIZONTAL, length=300, command=rx_slider_cb)
    rx_slider.set(0)
    rx_slider.pack()

    # For Z Right
    slider_label_hip_zr = tk.Label(right_frame, text="Z axis (Right)")
    slider_label_hip_zr.pack()

    rz_slider = tk.Scale(right_frame, from_=200, to=363, orient=tk.HORIZONTAL, length=300, command=rz_slider_cb)
    rz_slider.set(350)
    rz_slider.pack()

    # For Alpha Right
    slider_label_alphar = tk.Label(right_frame, text="Alpha (Right)")
    slider_label_alphar.pack()

    ralpha_slider = tk.Scale(right_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=ralpha_slider_cb)
    ralpha_slider.set(0)
    ralpha_slider.pack()

    # For Beta Right
    slider_label_betar = tk.Label(right_frame, text="Beta (Right)")
    slider_label_betar.pack()

    rbeta_slider = tk.Scale(right_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=rbeta_slider_cb)
    rbeta_slider.set(0)
    rbeta_slider.pack()

    # For Gamma Right
    slider_label_gammar = tk.Label(right_frame, text="Gamma (Right)")
    slider_label_gammar.pack()

    rgamma_slider = tk.Scale(right_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=rgamma_slider_cb)
    rgamma_slider.set(-15)
    rgamma_slider.pack()

    # Left Leg Sliders (Place them in the left_frame)
    slider_label_hip_25 = tk.Label(left_frame, text="(Hip - Left)")
    slider_label_hip_25.pack()

    ly_slider = tk.Scale(left_frame, from_=-200, to=200, orient=tk.HORIZONTAL, length=300, command=ly_slider_cb)
    ly_slider.set(0)
    ly_slider.pack()

    # For X Left
    slider_label_hip_xl = tk.Label(left_frame, text="X axis (Left)")
    slider_label_hip_xl.pack()

    lx_slider = tk.Scale(left_frame, from_=-200, to=200, orient=tk.HORIZONTAL, length=300, command=lx_slider_cb)
    lx_slider.set(0)
    lx_slider.pack()

    # For Z Left
    slider_label_hip_zl = tk.Label(left_frame, text="Z axis (Left)")
    slider_label_hip_zl.pack()

    lz_slider = tk.Scale(left_frame, from_=200, to=363, orient=tk.HORIZONTAL, length=300, command=lz_slider_cb)
    lz_slider.set(350)
    lz_slider.pack()

    # For Alpha Left
    slider_label_alphal = tk.Label(left_frame, text="Alpha (Left)")
    slider_label_alphal.pack()

    lalpha_slider = tk.Scale(left_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=lalpha_slider_cb)
    lalpha_slider.set(0)
    lalpha_slider.pack()

    # For Beta Left
    slider_label_betal = tk.Label(left_frame, text="Beta (Left)")
    slider_label_betal.pack()

    lbeta_slider = tk.Scale(left_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=lbeta_slider_cb)
    lbeta_slider.set(0)
    lbeta_slider.pack()

    # For Gamma Left
    slider_label_gammal = tk.Label(left_frame, text="Gamma (Left)")
    slider_label_gammal.pack()

    lgamma_slider = tk.Scale(left_frame, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=lgamma_slider_cb)
    lgamma_slider.set(-15)
    lgamma_slider.pack()

    # Buttons at the bottom
    tk.Button(button_frame, text="Do Seq", command=do_seq).pack(side=tk.LEFT)
    tk.Button(button_frame, text="Do Seq2", command=do_seq2).pack(side=tk.LEFT)
    tk.Button(button_frame, text="Stand Up", command=stand_up).pack(side=tk.LEFT)

    # Initialize the robot's leg parameters
    xr = 0
    yr = 0
    zr = 350
    alphaR = 0
    betaR = 0
    gammaR = -15

    xl = 0
    yl = 0
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -15

    # Initialize the robot's position
    move_robot()

    # Run the Tkinter event loop
    root.mainloop()


    '''
    time.sleep(2)
    xr = 0
    yr = -30
    zr = 350
    alphaR = 0
    betaR = 0
    gammaR = -15

    xl = 0
    yl = 30
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -15

    move_robot()

    time.sleep(3)

    xr = 100
    yr = -30
    zr = 330
    alphaR = 0
    betaR = 0
    gammaR = -15

    xl = 0
    yl = 30
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -15

    move_robot()

    time.sleep(10)


    
######### second step .....................................................................................
    time.sleep(2)
    xr = 100
    yr = 10
    zr = 330
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 0
    yl = -10
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -17

    move_robot()
    
    time.sleep(2)
    
    xr = 0
    yr = 10
    zr = 350
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 0
    yl = -10
    zl = 330
    alphaL = 0
    betaL = 0
    gammaL = -17

 

    move_robot()

    xr = 0
    yr = 10
    zr = 350
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 100
    yl = -10
    zl = 330
    alphaL = 0
    betaL = 0
    gammaL = -17

    move_robot() 
    
    time.sleep(1)

    xr = 100
    yr = 0
    zr = 330
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 0
    yl = 0
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -17

    move_robot()

    
    time.sleep(2)

    xr = 100
    yr = 5
    zr = 330
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 0
    yl = 0
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -17

    move_robot()
    time.sleep(1)

    xr = 100
    yr = 5
    zr = 330
    alphaR = 0
    betaR = 0
    gammaR = -10


    xl = 0
    yl = -5
    zl = 350
    alphaL = 0
    betaL = 0
    gammaL = -17

    move_robot()
    '''   

    
  
