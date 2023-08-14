#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import asyncio
import techmanpy
import time


def SpMax(Vr_Max, Vh_Max, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_Max*Ts + ((ac*pow(Ts, 2))/2)
    SpMax   = Vh_Max * (Tr + Ts) + (Vr_Max * Tr) + Ss + Ctot
    return SpMax

def Spmin(C, Zd, Zr):
    SpminVal = C + Zd + Zr
    return SpminVal

def SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C, Zd, Zr):
    global SpPFLVal
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts,2))/2)
    SpPFLVal   = Vh * ( Tr + Ts ) + (Vr_PFL * Tr) + Ss + Ctot
    return SpPFLVal

def SpSafe(Vr_PFL, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts, 2))/2)
    SpSafeVal = Ss + Ctot
    return SpSafeVal

def Vr_SSM(D, Vh, Tr, Ts, ac, C, Zd, Zr, Vr_PFL):
    T = Tr + Ts
    Ctot = C + Zd + Zr
    VrSSM = (((D - (Vh*T) - Ctot)) / T) - ((ac*pow(Ts, 2))/(2*T))
    if VrSSM < Vr_PFL:
        Reduce_Value = Vr_PFL
    else:
        Reduce_Value = VrSSM
    return Reduce_Value

def Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    VrSSM2 = (D / Ts) - ((ac*Ts)/2) - (Ctot/Ts)
    if VrSSM2 > 0:
        Stop_Value = VrSSM2
    else:
        Stop_Value = 0
    return Stop_Value

def remap(value, from_low, from_high, to_low, to_high):
    # Clamp the value within the from range
    clamped_value = max(from_low, min(value, from_high))
    # Map the clamped value to the to range
    mapped_value = (clamped_value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    return mapped_value



VrPaper = 1000
Vr = 1500
Vr_PFL = 400
Vh_max = 1600
Vh_min = 0
Tr = 0.1
Ts = 0.08
ac = 3000
C_SSM = 200
Zd = 106.7
Zr = 1
speedUpdate = 0

class RobotManipulator:
    def __init__(self):
        #self.pub = rospy.Publisher("Velocity_Feedback", Float32, queue_size=10)
        self.distance_subscribe = rospy.Subscriber('humanDistance', Float32, self.pose_callback)
        self.time_subscribe = rospy.Subscriber('Time', Float32, self.time_callback)
        rate = rospy.Rate(10)  # 10hz
        self.robSpeed = []  # List to store the readings
        self.plotTime = []  # List to store the readings

        # Set up the plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (mm)')
        self.ax.set_title('Distance Feedback Over Time')
        self.ax.grid(True)

        # Set initial axis limits
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 1000)


    def pose_callback(self, data):
        global speedUpdate
        Vr = 1500
        self.SpminVal = Spmin(C_SSM, Zd, Zr)
        self.SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)
        self.SpPFLVal = SpPFL(Vr_PFL, Vh_max, Tr, Ts, ac, C_SSM, Zd, Zr)
        self.SpMaxVal = SpMax(Vr, Vh_max, Tr, Ts, ac, C_SSM, Zd, Zr)
        D = data.data - 400
        if  D <= self.SpminVal:
            print("Robot Pause")
            speedUpdate = 0
            Vr = 0
            print("Velocity Feedback Zero: ", speedUpdate)
    
        elif D > self.SpminVal and D <= self.SpSafeVal:
            #await conn.resume_project()
            Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
            Vr = round(Vr, 2)
            speedUpdate = round(remap(Vr, 0, 1500, 0, 1), 2)
            print("Velocity Feedback Slow Down to Stop: ", speedUpdate)
        
        elif D > self.SpSafeVal and D <= self.SpPFLVal:
            Vr = Vr_PFL
            Vr = round(Vr, 2)
            speedUpdate = round(remap(Vr, 0, 1500, 0, 1), 2)
            print("Velocity Feedback Buffer PFL: ", speedUpdate)
        
        elif D > self.SpPFLVal and D <= self.SpMaxVal:
            Vr = Vr_SSM(D, Vh_max, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
            Vr = round(Vr, 2)
            speedUpdate = round(remap(Vr, 0, 1500, 0, 1), 2)
            print("Velocity Feedback Reduce Speed: ", speedUpdate)
            
        else:
            Vr = 1500
            speedUpdate = round(remap(Vr, 0, 1500, 0, 1), 2)
            print("Velocity Feedback Full Speed: ", speedUpdate)
            

        self.robSpeed.append(speedUpdate)  # Store the readings in the list
        #self.pub.publish(reading_velocity)

    def time_callback(self, data):
        print("Time received data: ", data.data)
        self.plotTime.append(data.data)  # Store the readings in the list


    def init_animation(self):
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        xdata = self.plotTime
        ydata = self.robSpeed
        self.line.set_data(xdata, ydata)

        # Manually set the x-axis and y-axis limits
        self.ax.set_xlim(0, max(self.plotTime))
        self.ax.set_ylim(0, max(self.robSpeed) + 1)

        return self.line,

    async def mode_robot(self):
        #global speedUpdate
        postMove = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        #async with techmanpy.connect_sct(robot_ip='192.168.10.2') as conn:
        print(speedUpdate)
        for i in postMove:
            while speedUpdate <= 0:  # Pause the loop if 'paused' is True
                #await conn.pause_project()
                print("Loop paused...")
            #movement1
            #for move in list move
                #speed
            #await conn.resume_project()
            #await conn.move_to_point_ptp([268.65, 280.21, 405.36, -179.42, 1.12, 175.92], speedUpdate, 200)
            print("Loop resume...")
            print("Robot berhasil diupdate kecepatan ", speedUpdate)
            print("Gerakan robot ke-", i)
            asyncio.sleep(3)
            #while variable check condition resume or pause
        #await conn.move_to_point_ptp([-243.41, 368.41, 405.61, 175.97, -2.70, -175.92], 0.20, 200)
        #if finish all movement -> counter+1 -> publish into the camera+GUI node 

if __name__ == '__main__':
    rospy.init_node('robot_receiver', anonymous=True)
    robot = RobotManipulator()
    asyncio.run(robot.mode_robot())
    ani = FuncAnimation(robot.fig, robot.update_plot, init_func=robot.init_animation, interval=100)

    plt.show()
