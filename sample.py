import vrep
import time
import numpy as np
import sys, termios, tty, os, time
import csv

pose_list = []

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def fetchKinect(depthSTR,rgbSTR):
    errorCodeKinectRGB,kinectRGB=vrep.simxGetObjectHandle(clientID,rgbSTR,vrep.simx_opmode_oneshot_wait)
    errorCodeKinectDepth,kinectDepth=vrep.simxGetObjectHandle(clientID,depthSTR,vrep.simx_opmode_oneshot_wait)


    errorHere,resolution,image=vrep.simxGetVisionSensorImage(clientID,kinectRGB,0,vrep.simx_opmode_oneshot_wait)
    # img,imgArr=Raftaar.ProcessImage(image,resolution)
    # rgbArr=np.array(imgArr)

    errorHere,resol,depth=vrep.simxGetVisionSensorDepthBuffer(clientID,kinectDepth,vrep.simx_opmode_oneshot_wait)
    depthArr=np.array(depth)


    print(resol)

    #fetchKinect('kinect_depth','kinect_rgb')

def flyObject(z_vel, x_vel):
    t_end = time.time() + 5
    z_vel = float(z_vel)
    x_vel = float(x_vel)
    while time.time() < t_end:

        res1, obj1 = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
        vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_z, 0, vrep.simx_opmode_blocking)
        vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_x, 0, vrep.simx_opmode_blocking)

def getPosition(clientID):
    res, obj = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
    pose = vrep.simxGetObjectPosition(clientID, obj, -1, vrep.simx_opmode_blocking)


    return pose

def calc_parabola_vertex(x1, y1, x2, y2, x3, y3):
    denom = (x1-x2) * (x1-x3) * (x2-x3)
    a = (x3 * (y2-y1) + x2 * (y1-y3) + x1 * (y3-y2)) / denom
    b = (x3*x3 * (y1-y2) + x2*x2 * (y3-y1) + x1*x1 * (y2-y3)) / denom
    c = (x2 * x3 * (x2-x3) * y1+x3 * x1 * (x3-x1) * y2+x1 * x2 * (x1-x2) * y3) / denom
    return a,b,c

if __name__ == "__main__":


    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

    while True:
        char = getch()


        if (char == "m"):
            print("Stop!")
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        if (char == "s"):
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
            exit(0)

        if (char == "p"):
            vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)


        xx = []
        zz = []
        x_req = 0.01

        if (char == "n"):
            res1, obj1 = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
            z_vel, x_vel = input("velocity in X and Y: ")
            vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_z, z_vel, vrep.simx_opmode_blocking)
            vrep.simxSetObjectFloatParameter(clientID, obj1, vrep.sim_shapefloatparam_init_velocity_x, x_vel, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
            t_end = time.time() + 5
            while time.time() < t_end:
                pose = getPosition(clientID)
                pose_list.append(pose)
                if len(pose_list) > 2 & len(pose_list) <6:
                    xyz = pose[1]
                    xx.append(xyz[0])
                    zz.append(xyz[2])

                if len(pose_list) >6:
                    a,b,c = calc_parabola_vertex(xx[0], zz[0], xx[1], zz[1], xx[2], zz[2])
                    z_req= (a*(x_req**2))+(b*x_req)+c
                    print(z_req)

                    if pose[1][0] >= -0.05:

                        vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot_wait)


            #print(pose_list)

            with open('poses.csv','wb') as out:
                csv_out=csv.writer(out)
                csv_out.writerow(['status','XYZ'])
                for row in pose_list:
                    csv_out.writerow(row)
            #vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)