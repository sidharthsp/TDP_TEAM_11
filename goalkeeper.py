"""main controller."""
# Author:Boyu Shi
#
# Copyright 2023 Boyu Shi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np
from controller import Robot
from controller import Accelerometer
from controller import Camera
from controller import Motor
from controller import Motion
from controller import Receiver
import math
import _thread
import random
import time

Robot_num = 4

goal_x_width = 2.6
goal_y_width = 1
goal_dis = 4.5
bottom_line_width=6

rad_robust=0.5

swerve_x_pos=0.75

class Nao(Robot):
    # load motion files
    def loadMotionFiles(self):
        self.move = Motion('../../motions/Move.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.SideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.SideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft30 = Motion('../../motions/TurnLeft30.motion')
        self.turnRight30 = Motion('../../motions/TurnRight30.motion')
        self.shoot = Motion('../../motions/Shoot.motion')
        self.stand = Motion('../../motions/Stand2.motion')
        self.standup = Motion('../../motions/StandUpFromFront.motion')
        self.DiveLeft = Motion('../../motions/DiveLeft.motion')
        self.DiveRight = Motion('../../motions/DiveRight.motion')

    def set_motion_time_direc(self):
        self.motion_time_direc={
            "move":2.6,
            "backwards":2.6,
            "SideStepLeft":4.92,
            "SideStepRight":5.76,
            "turnLeft30":4.52,
            "turnRight30":4.52,
            "shoot":1.16,
            "standup":3.7,
            "DiveLeft":2.0,
            "DiveRight":2.0,
        }

    def startMotion(self, motion):
        # interrupt current motion
        #print(str(self.rob_name)+"start motion:"+str(self.currentlyPlaying)+"next motion:"+str(motion))
        self.motion_start_time=self.getTime()
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)
        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # receiver
        receiver_name = self.name + "r"
        self.receiver = self.getDevice(receiver_name)

        # GPS
        self.gps = self.getDevice(self.getName() + "gps")
        self.gps.enable(self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice(self.getName() + "IU")
        self.inertialUnit.enable(self.timeStep)

    def receive_meaasge(self):#Boyu Shi
        #get the message from supervisor“Judge”
        flag = 0
        if (self.receiver.getQueueLength() > 0):
            message = self.receiver.getFloats()
            length = len(message)
            # print(str(self.rob_name)+message)
            epoch_size = (Robot_num * 2 + 1) * 3
            epoch = length / epoch_size
            self.newest = []
            #each package of message may contain several packages position message of robots and ball, we only need newest one
            for i in range(int((epoch - 1) * epoch_size), int(epoch * epoch_size), 3):
                pos = [message[i], message[i + 1], message[i + 2]]
                if (self.first_pos == 0):
                    self.newest.append(pos)
                else:
                    self.newest[int(i / 3)] = pos
            flag = 1
            self.receiver.nextPacket()
        return flag

    def count_dis(self):
        pos=self.gps.getValues()
        for i in range(Robot_num*2):
            if(robotlist[i]!=self.rob_name):
                self.dis[i]=math.sqrt((pos[0]-self.newest[i][0])**2+(pos[1]-self.newest[i][1])**2)
            else:
                self.dis[i]=0
        return
    def nearest_teammate(self):
        bias=0
        if(self.team=='B'):
            bias=1
        nearst_idx=0
        nearst_dis=-1
        for i in range(3):
            if(robotlist[i*2+bias]==self.rob_name):
                continue
            if(nearst_dis==-1):
                nearst_dis=self.dis[i*2+bias]
            else:
                if(self.dis[i*2+bias]<nearst_dis):
                    nearst_dis=self.dis[i*2+bias]
                    nearst_idx=i*2+bias
        return self.newest[nearst_idx]


    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        # initialize
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.receiver.enable(20)
        self.first_pos = 0  # for receiver
        self.ZERO_ANGLE = 0
        self.CATCH_BALL = False  # for shoot and bring ball forward
        self.ISDETECTING = False  # detecting ball pos and robot's face direction
        self.ISBACK = False  # for direction and position check
        self.IFSHOOT = False
        self.rob_name = self.getName()
        self.team = self.rob_name[0]
        self.DIR=False#if robot is face to target position
        self.TURNING=False#if robot is turning around
        self.target_pos=[]
        self.CHANGE_TARGET=False
        self.activate_motion=None
        self.motion_start_time=0
        self.target_is=-1 #0-> target is ball, 1-> target is another point,-1->target need initialise
        self.set_motion_time_direc()#make the motion-cost time dictionaries
        self.dis=np.zeros(Robot_num*2)

    def getAngle(self, target_Pos, Pos, RollPitchYaw):#Ziyuan Liu
        #get the angle between robot's face direction and ball's postion
        delta_x = target_Pos[0] - Pos[0]
        delta_y = target_Pos[1] - Pos[1]
        sin = delta_y / math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        turn_angle = math.asin(sin)

        if turn_angle < 0:
            if delta_x < 0:
                turn_angle = - turn_angle - math.pi
        else:
            if delta_x < 0:
                turn_angle = math.pi - turn_angle

        beta = RollPitchYaw[2]
        Angle = turn_angle - beta

        if Angle < -math.pi:
            Angle += 2 * math.pi
        if Angle > math.pi:
            Angle -= 2 * math.pi

        return Angle

    def if_dir(self,target_Pos, Pos, RollPitchYaw):#Ziyuan Liu,Boyu Shi
        #check if robot is directly face to the target position
        self.DIR=False
        delta_x = target_Pos[0] - Pos[0]
        delta_y = target_Pos[1] - Pos[1]
        sin = delta_y / math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        turn_angle = math.asin(sin)

        if turn_angle < 0:
            if delta_x < 0:
                turn_angle = - math.pi - turn_angle
        else:
            if delta_x < 0:
                turn_angle = math.pi - turn_angle

        beta = RollPitchYaw[2]
        Angle = turn_angle - beta

        if Angle < -math.pi:
            Angle += 2 * math.pi
        if Angle > math.pi:
            Angle -= 2 * math.pi
        if abs(Angle)<rad_robust:
            self.DIR=True

        return

    def detectAngle(self, init_face_dir, angle):#Ziyuan Liu
        #detect if rebot is directly face to the ball,if true stop turning around
        while self.ISDETECTING:
            print(str(self.rob_name)+"is detecting")
            present_face_dir = self.inertialUnit.getRollPitchYaw()
            temp = abs(present_face_dir[2] - init_face_dir[2] - angle)
            if temp < rad_robust:
                if self.currentlyPlaying:
                    self.currentlyPlaying.stop()
                    self.activate_motion=None
                self.ISDETECTING = False
                self.DIR=True
                print(str(self.rob_name)+"direction correct now")
            return

    def turnAround(self, taeget_pos):#Ziyuan Liu
        #if rebot is not directly face to the ball start turning around
        Pos = self.gps.getValues()
        init_face_dir = self.inertialUnit.getRollPitchYaw()
        angle = self.getAngle(taeget_pos, Pos, init_face_dir)
        if abs(angle) >= rad_robust:
            if not self.ISDETECTING:
                self.ISDETECTING = True
                self.DIR = False
                #print(str(self.rob_name)+"turnAround:" + str(robot.IFMOVE))
                _thread.start_new_thread(self.detectAngle, (init_face_dir, angle))
            if angle > rad_robust:
                self.activate_motion='turnLeft30'
                self.startMotion(self.turnLeft30)
            else:
                self.activate_motion='turnRight30'
                self.startMotion(self.turnRight30)
        return

    def turn_label(self, target_pos):#Ziyuan Liu
        #start a new thread to check and change direction
        self.ISDETECTING = False
        self.DIR=True
        _thread.start_new_thread(self.turnAround, (target_pos,))
        #print(str(self.rob_name)+"turn_label:"+str(robot.IFMOVE))

    def if_catch_ball(self, ballPos):#Boyu Shi
        #check if robot is close enough to ball
        Pos = self.gps.getValues()
        ball_dis = math.sqrt((ballPos[0] - Pos[0]) ** 2 + (ballPos[1] - Pos[1]) ** 2)
        if ball_dis < 0.2:
            self.CATCH_BALL = True
        else:
            self.CATCH_BALL = False
        return

    def ifshoot(self, ballPos):#Boyu Shi
        #check if the ball is in a suitable position to shoot
        self.IFSHOOT = False
        if self.team == 'R':
            if -goal_x_width/2 < ballPos[0] < goal_x_width/2 and goal_dis-1 <= ballPos[1] <= goal_dis:
                self.IFSHOOT = True
        elif self.team == 'B':
            if -goal_x_width/2 < ballPos[0] < goal_x_width/2 and -goal_dis < ballPos[1] <= 1 - goal_dis:
                self.IFSHOOT = True
        else:
            print(str(self.rob_name)+"Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        return

    def if_change_target(self,previous_pos,now_pos):
        self.CHANGE_TARGET=False
        if self.team == 'B':
            if previous_pos[1]>now_pos[1]:
                self.CHANGE_TARGET=True
        elif self.team == 'R':
            if previous_pos[1]<now_pos[1]:
                self.CHANGE_TARGET=True
        else:
            print(str(self.rob_name)+"Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        Pos = robot.gps.getValues()
        if math.sqrt((Pos[0]-previous_pos[0])**2+(Pos[1]-previous_pos[1])**2)<0.2:
            self.CHANGE_TARGET=True
        return self.CHANGE_TARGET

    def check_belong(self):#belong_team 1->blue 2->red
        ball_pos = self.newest[2 * Robot_num]
        blue_control = 0
        red_control = 0
        control_rob=-1
        for i in range(2 * Robot_num):
            if (math.sqrt((self.newest[i][0] - ball_pos[0]) ** 2 + (
                    self.newest[i][1] - ball_pos[1]) ** 2)) < 0.4:
                control_rob=i
                if (i % 2 == 0):
                    red_control += 1
                else:
                    blue_control += 1
            if (red_control and blue_control):
                break
        if (blue_control != 0 and red_control == 0):
            self.belong_team = 1
        elif (blue_control == 0 and red_control != 0):
            self.belong_team = 2
        else:
            self.belong_team = 0
            control_rob=-1
        return control_rob

    def in_goal_area(self):
        in_goal_area=False
        if robot.team=='R':
            if -goal_x_width/2<=self.newest[Robot_num*2][0]<=goal_x_width/2 and -goal_dis<=self.newest[Robot_num*2][1]<=-goal_dis+1:
                in_goal_area=True
        elif robot.team=='B':
            if -goal_x_width/2<=self.newest[Robot_num*2][0]<=goal_x_width/2 and goal_dis-1<=self.newest[Robot_num*2][1]<=goal_dis:
                in_goal_area=True
        return in_goal_area

    def if_front_door(self):
        front_door=0#1 step_left 2 step_right 3 step_forward 4 step_back
        pos=self.gps.getValues()
        if robot.team=='R':
            if pos[0]>swerve_x_pos:
                front_door=1
            elif pos[0]<-swerve_x_pos:
                front_door=2
            elif pos[1]<-goal_dis:
                front_door=3
            elif pos[1]>-goal_dis+0.3:
                front_door=4
        elif robot.team=='B':
            if pos[0]>swerve_x_pos:
                front_door=2
            elif pos[0]<-swerve_x_pos:
                front_door=1
            elif pos[1]>goal_dis:
                front_door=3
            elif pos[1]<goal_dis-0.3:
                front_door=4
        return front_door

    def tuneLR(self,x_t,Pos,ifdive):
        if self.team=="R":
            if (math.fabs(x_t - Pos[0]) > 0.5):
                if (x_t < Pos[0] and Pos[0]> -swerve_x_pos):
                    self.activate_motion = "SideStepLeft"
                    self.startMotion(self.SideStepLeft)
                elif x_t > Pos[0] and Pos[0]< swerve_x_pos:
                    self.activate_motion = "SideStepRight"
                    self.startMotion(self.SideStepRight)
            elif ifdive==1:
                if (x_t <= Pos[0]):
                    self.activate_motion = "DiveLeft"
                    self.startMotion(self.DiveLeft)
                else:
                    self.activate_motion = "DiveRight"
                    self.startMotion(self.DiveRight)
        elif self.team=="B":
            if (math.fabs(x_t - Pos[0]) > 0.5):
                if (x_t > Pos[0] and Pos[0] < swerve_x_pos):
                    self.activate_motion = "SideStepLeft"
                    self.startMotion(self.SideStepLeft)
                elif x_t < Pos[0] and Pos[0]> -swerve_x_pos:
                    self.activate_motion = "SideStepRight"
                    self.startMotion(self.SideStepRight)
            elif ifdive==1:
                if (x_t >= Pos[0]):
                    self.activate_motion = "DiveLeft"
                    self.startMotion(self.DiveLeft)
                else:
                    self.activate_motion = "DiveRight"
                    self.startMotion(self.DiveRight)
        return
    def x_t_calculate(self,control_rob):
        if robot.newest[control_rob][0]==robot.newest[Robot_num*2][0]:
            x_t=robot.newest[control_rob][0]
        else:
            if (robot.team == 'R'):
                if (robot.newest[control_rob][1] - robot.newest[Robot_num * 2][1])!=0:
                    k = (robot.newest[control_rob][1] - robot.newest[Robot_num * 2][1]) / (
                            robot.newest[control_rob][0] - robot.newest[Robot_num * 2][0])
                else:
                    k=0
                b = robot.newest[control_rob][1] - k * robot.newest[control_rob][0]
                if k!=0:
                    x_t = (-goal_dis - b) / k
                else:
                    x_t=robot.newest[Robot_num*2][0]
            elif (robot.team == 'B'):
                if (robot.newest[control_rob][1] - robot.newest[Robot_num * 2][1])!=0:
                    k = (robot.newest[control_rob][1] - robot.newest[Robot_num * 2][1]) / (
                            robot.newest[control_rob][0] - robot.newest[Robot_num * 2][0])
                else:
                    k=0
                b = robot.newest[control_rob][1] - k * robot.newest[control_rob][0]
                if k!=0:
                    x_t = (goal_dis - b) / k
                else:
                    x_t=robot.newest[Robot_num*2][0]
        return x_t


robot = Nao()

robotlist = []


def initial_robotlist():#Boyu Shi
    #list name of robots
    for i in range(Robot_num):
        robotlist.append("R" + str(i))
        robotlist.append("B" + str(i))


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
initial_robotlist()
# Main loop:
# - perform simulation steps until Webots is stopping the controller

#main written by Boyu Shi
robot.target_pos=robot.gps.getValues()
robot.stand.play()
while robot.step(timestep) != -1:
    if (robot.receive_meaasge()):
        ball_pos = robot.newest[8]
        Pos=robot.gps.getValues()
        '''
        if(Pos[2]<0.2):
            robot.activate_motion='standup'
            robot.startMotion(robot.standup)
        '''
        if(robot.activate_motion!=None):
            present_time=robot.getTime()
            delta_time=present_time-robot.motion_start_time
            if(not delta_time>=robot.motion_time_direc[robot.activate_motion]):
               continue
            else:
                robot.currentlyPlaying.stop()
                robot.activate_motion=None
        #make robot face to the pitch
        if robot.team=='R':
            robot.target_pos=[Pos[0],Pos[1]+0.3]
        elif robot.team=='B':
            robot.target_pos=[Pos[0],Pos[1]-0.3]
        else:
            print(str(robot.rob_name)+"Uncorrectly name the player:" + robot.rob_name + "You need to set team name R or B first")
        present_face_dir = robot.inertialUnit.getRollPitchYaw()
        robot.if_dir(robot.target_pos, Pos, present_face_dir)
        if (robot.DIR):#if robot face to the pitch
            move_dir=robot.if_front_door()
            if(move_dir==0):#if robot is in front of door
                control_rob = robot.check_belong()
                if(robot.in_goal_area()):#if robot is in the goal area
                    if(control_rob!=-1):#if someone control the ball
                        if(robot.team=='R'):
                            if(robot.belong_team==1):#if opposing player in possession of the ball, move to the corresponding position and pounce
                                if(robot.newest[control_rob][1]>ball_pos[1]):
                                    x_t=robot.x_t_calculate(control_rob)
                                    robot.tuneLR(x_t,Pos,1)
                                else:
                                    robot.tuneLR(ball_pos[0], Pos, 0)
                            else:
                                robot.tuneLR(ball_pos[0],Pos,0)
                        elif(robot.team=='B'):
                            if (robot.belong_team == 0):
                                if (robot.newest[control_rob][1] < ball_pos[1]):
                                    x_t = robot.x_t_calculate(control_rob)
                                    robot.tuneLR(x_t, Pos, 1)
                                else:
                                    robot.tuneLR(ball_pos[0], Pos, 0)
                            else:
                                robot.tuneLR(ball_pos[0],Pos,0)
                    else:#if no one control the ball
                        robot.tuneLR(ball_pos[0],Pos,0)
                else:#if ball is not in the goal area
                    if (control_rob != -1):
                        if (robot.team == 'R'):
                            if (robot.belong_team == 1):#if opposing player in possession of the ball,move to prepare to defend
                                if (robot.newest[control_rob][1] > ball_pos[1]):
                                    x_t = robot.x_t_calculate(control_rob)
                                    robot.tuneLR(x_t, Pos, 0)
                                else:
                                    robot.tuneLR(ball_pos[0], Pos, 0)
                            else:
                                robot.tuneLR(ball_pos[0],Pos,0)
                        elif (robot.team == 'B'):
                            if (robot.belong_team == 0):
                                if (robot.newest[control_rob][1] < ball_pos[1]):
                                    x_t = robot.x_t_calculate(control_rob)
                                    robot.tuneLR(x_t, Pos, 0)
                                else:
                                    robot.tuneLR(ball_pos[0], Pos, 0)
                            else:
                                robot.tuneLR(ball_pos[0],Pos,0)
                    else:#if no one control the ball
                        robot.tuneLR(ball_pos[0],Pos,0)
            else:#if goalkeeper is not in front of the door
                if move_dir==1:
                    robot.activate_motion = "SideStepLeft"
                    robot.startMotion(robot.SideStepLeft)
                elif move_dir==2:
                    robot.activate_motion = "SideStepRight"
                    robot.startMotion(robot.SideStepRight)
                elif move_dir==3:
                    robot.activate_motion = "move"
                    robot.startMotion(robot.move)
                elif move_dir==4:
                    robot.activate_motion = "backwards"
                    robot.startMotion(robot.backwards)
        else:#if goalkeeper don't face to the pitch
            robot.turn_label(robot.target_pos)








"""

while robot.step(timestep) != -1:
    robot.startMotion(robot.DiveLeft)
"""
