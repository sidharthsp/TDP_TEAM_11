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

rad_robust=0.2

class Nao(Robot):

    # load motion files
    def loadMotionFiles(self):
        self.move = Motion('../../motions/Move.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.shoot = Motion('../../motions/Shoot.motion')
        self.taichi = Motion('../../motions/Taichi.motion')

    def startMotion(self, motion):
        # interrupt current motion
        #print("start motion:"+str(self.currentlyPlaying)+"nexxt motion:"+str(motion))
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

    def receive_meaasge(self):
        flag = 0
        if (self.receiver.getQueueLength() > 0):
            message = self.receiver.getFloats()
            length = len(message)
            # print(message)
            epoch_size = (Robot_num * 2 + 1) * 3
            epoch = length / epoch_size
            self.newest = []
            for i in range(int((epoch - 1) * epoch_size), int(epoch * epoch_size), 3):
                pos = [message[i], message[i + 1], message[i + 2]]
                if (self.first_pos == 0):
                    self.newest.append(pos)
                else:
                    self.newest[int(i / 3)] = pos
            flag = 1
            self.receiver.nextPacket()
        return flag

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
        self.DIR=False

    def getAngle(self, target_Pos, Pos, RollPitchYaw):
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

    def if_dir(self,target_Pos, Pos, RollPitchYaw):
        self.DIR=False
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

        if abs(Angle)<rad_robust:
            self.DIR=True

        return

    def detectAngle(self, init_face_dir, angle):
        while True:
            present_face_dir = self.inertialUnit.getRollPitchYaw()
            temp = abs(present_face_dir[2] - init_face_dir[2] - angle)
            if temp < rad_robust:
                if self.currentlyPlaying:
                    self.currentlyPlaying.stop()
                    self.currentlyPlaying = None
                self.ISDETECTING = False
                self.DIR=True
                print("direction correct now")
                return

    def turnAround(self, ballPos):
        Pos = self.gps.getValues()
        init_face_dir = self.inertialUnit.getRollPitchYaw()
        angle = self.getAngle(ballPos, Pos, init_face_dir)
        if abs(angle) >= rad_robust:
            if not self.ISDETECTING:
                print("start turn around")
                self.ISDETECTING = True
                self.DIR = False
                #print("turnAround:" + str(robot.IFMOVE))
                _thread.start_new_thread(self.detectAngle, (init_face_dir, angle))
            if angle > rad_robust:
                self.startMotion(self.turnLeft60)
            else:
                self.startMotion(self.turnRight60)
        else:
            self.DIR=True
        return

    def turn_label(self, ballPos):
        self.ISDETECTING = False
        self.DIR=False
        _thread.start_new_thread(self.turnAround, (ballPos,))
        #print("turn_label:"+str(robot.IFMOVE))

    def if_catch_ball(self, ballPos):
        Pos = self.gps.getValues()
        ball_dis = math.sqrt((ballPos[0] - Pos[0]) ** 2 + (ballPos[1] - Pos[1]) ** 2)
        if ball_dis < 0.2:
            self.CATCH_BALL = True
        else:
            self.CATCH_BALL = False
        return

    def goal_dir(self):
        xx = random.uniform(-goal_x_width / 2, goal_x_width / 2)
        selfpos = self.gps.getValues()
        if self.team == 'B':
            target_pos = [xx, -goal_dis, 0]
        elif self.team == 'R':
            target_pos = [xx, goal_dis, 0]
        else:
            print("Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        dir = self.getAngle(target_pos, selfpos, self.inertialUnit.getRollPitchYaw())
        return dir

    def ifshoot(self, ballPos):
        self.IFSHOOT = False
        if self.team == 'R':
            if -goal_x_width/2 < ballPos[0] < goal_x_width/2 and goal_dis-1 <= ballPos[1] <= goal_dis:
                self.IFSHOOT = True
        elif self.team == 'B':
            if -goal_x_width/2 < ballPos[0] < goal_x_width/2 and -goal_dis < ballPos[1] <= 1 - goal_dis:
                self.IFSHOOT = True
        else:
            print("Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        return self.IFSHOOT

    def isback(self, ballPos):
        turn_dir = 'r'
        selfpos = self.gps.getValues()
        if self.team == 'B':
            k1 = (-goal_dis - ballPos[1]) / ((goal_x_width - 0.2) / 2 - ballPos[0])
            k2 = (-goal_dis - ballPos[1]) / (-(goal_x_width - 0.2) / 2 - ballPos[0])
            k3 = (ballPos[1]-selfpos[1])/(ballPos[0]-selfpos[0])
            b1 = ballPos[1] - k1 * ballPos[0]#-0.3/math.sin(math.atan(k1))
            b2 = ballPos[1] - k2 * ballPos[0]#-0.3/math.sin(math.atan(k2))
            b3 = selfpos[1]-k3*selfpos[0]
            if -goal_x_width/2<(4.5-b3)/k3<goal_x_width/2 and selfpos[1]>ballPos[1]:
                self.ISBACK = True
            elif selfpos[0] * k1 + b1 > selfpos[1] and selfpos[0] * k2 + b2 < selfpos[1]:
                turn_dir = 'l'
                print("circle_turn_left")

        elif self.team == 'R':
            k1 = (goal_dis - ballPos[1]) / ((goal_x_width - 0.2) / 2 - ballPos[0])
            k2 = (goal_dis - ballPos[1]) / (-(goal_x_width - 0.2) / 2 - ballPos[0])
            k3 = (ballPos[1] - selfpos[1]) / (ballPos[0] - selfpos[0])
            b1 = ballPos[1] - k1 * ballPos[0]#+0.3/math.sin(math.atan(k1))
            b2 = ballPos[1] - k2 * ballPos[0]#+0.3/math.sin(math.atan(k2))
            b3 = selfpos[1] - k3 * selfpos[0]
            print("(goal_dis-b3)/k3="+str((goal_dis-b3)/k3))
            print("selfpos[1]="+str(selfpos[1]))
            print("ballPos[1]="+str(ballPos[1]))
            if -goal_x_width/2<(goal_dis-b3)/k3<goal_x_width/2 and selfpos[1]<ballPos[1]:
                self.ISBACK = True
            elif selfpos[0] * k1 + b1 > selfpos[1] and selfpos[0] * k2 + b2 < selfpos[1]:
                turn_dir = 'l'
                print("circle_turn_left")
        else:
            print("Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        return turn_dir

    def circle_turn(self, turn_dir):
        if turn_dir == 'r':
            self.startMotion(self.sideStepRight)
        elif turn_dir == 'l':
            self.startMotion(self.sideStepLeft)
        else:
            print("wrong input for circle turn")
        return


robot = Nao()

robotlist = []


def initial_robotlist():
    for i in range(Robot_num):
        robotlist.append("R" + str(i))
        robotlist.append("B" + str(i))


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # get the position of ball(and other robots)
    if (robot.receive_meaasge()):
        ball_pos = robot.newest[8]
        Pos = robot.gps.getValues()
        init_face_dir = robot.inertialUnit.getRollPitchYaw()
        robot.if_dir(ball_pos, Pos, init_face_dir)
        if(robot.DIR):
            #"""
            robot.if_catch_ball(ball_pos)
            if(robot.CATCH_BALL):

                ib=robot.isback(ball_pos)
                if(robot.ISBACK):
                    if(robot.ifshoot(ball_pos)):
                        robot.startMotion(robot.shoot)
                else:
                    robot.circle_turn(ib)
                #"""
        else:
            if(robot.IFSHOOT):
                robot.IFSHOOT=False
            robot.turn_label(ball_pos)
    #print("before move:"+str(robot.IFMOVE))
    if robot.DIR and not (not robot.ISBACK and robot.CATCH_BALL) and not robot.IFSHOOT:
        print("move forward")
        robot.startMotion(robot.move)

# Enter here exit cleanup code.
