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

R_robot = 0.2

goal_x_width = 2.6
goal_y_width = 1
goal_dis = 4.5
bottom_line_width = 6

rad_robust = 0.25


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
        self.stand = Motion('../../motions/Stand.motion')
        self.standup = Motion('../../motions/StandUpFromFront.motion')

    def set_motion_time_direc(self):
        self.motion_time_direc = {
            "move": 2.6,
            "backwards": 2.6,
            "SideStepLeft": 4.92,
            "SideStepRight": 5.76,
            "turnLeft30": 4.52*2,
            "turnRight30": 4.52*2,
            "shoot": 1.16,
            "standup": 3.7,
        }

    def startMotion(self, motion):
        # interrupt current motion
        # print(str(self.rob_name)+"start motion:"+str(self.currentlyPlaying)+"next motion:"+str(motion))
        self.motion_start_time = self.getTime()
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

    def receive_message(self):  # Boyu Shi
        # get the message from supervisor“Judge”
        flag = 0
        if (self.receiver.getQueueLength() > 0):
            message = self.receiver.getFloats()
            length = len(message)
            # print(str(self.rob_name)+message)
            epoch_size = (Robot_num * 2 + 1) * 3
            epoch = length / epoch_size
            self.newest = []
            # each package of message may contain several packages position message of robots and ball, we only need newest one
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
        self.IFSHOOT = False
        self.rob_name = self.getName()
        self.team = self.rob_name[0]
        self.DIR = False  # if robot is face to target position
        self.target_pos = self.gps.getValues()
        self.CHANGE_TARGET = False
        self.activate_motion = None
        self.motion_start_time = 0
        self.target_is = -1  # 0-> target is ball, 1-> target is another point,-1->target need initialise
        self.set_motion_time_direc()  # make the motion-cost time dictionaries
        self.obstacle = np.zeros(Robot_num * 2)
        self.truningangle=10000

    def getAngle(self, target_Pos, Pos, RollPitchYaw):  # Ziyuan Liu
        # get the angle between robot's face direction and ball's postion
        delta_x = target_Pos[0] - Pos[0]
        delta_y = target_Pos[1] - Pos[1]
        if delta_x == 0 and delta_y == 0:
            sin = 1
        else:
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

    def if_dir(self, target_Pos, Pos, RollPitchYaw):  # Ziyuan Liu,Boyu Shi
        # check if robot is directly face to the target position
        self.DIR = False
        delta_x = target_Pos[0] - Pos[0]
        delta_y = target_Pos[1] - Pos[1]
        if delta_x == 0 and delta_y == 0:
            sin = 1
        else:
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
        if abs(Angle) < rad_robust:
            self.DIR = True

        return

    def detectAngle(self, init_face_dir,taeget_pos):  # Ziyuan Liu
        # detect if rebot is directly face to the ball,if true stop turning around
        while self.ISDETECTING:
            present_face_dir = self.inertialUnit.getRollPitchYaw()
            temp = abs(present_face_dir[2] - init_face_dir[2] - self.truningangle)
            #print(str(self.rob_name) + "is detecting\n")
            #print("temp:" + str(temp) + '\n')
            if temp < rad_robust:
                if self.currentlyPlaying:
                    self.currentlyPlaying.stop()
                    self.activate_motion = None
                self.ISDETECTING = False
                self.DIR = True
                print(str(self.rob_name) + "direction correct now")
        return


    def turnAround(self, taeget_pos):  # Ziyuan Liu
        # if rebot is not directly face to the ball start turning around
        Pos = self.gps.getValues()
        init_face_dir = self.inertialUnit.getRollPitchYaw()
        self.truningangle = self.getAngle(taeget_pos, Pos, init_face_dir)
        if (abs(self.truningangle) >= rad_robust):
            if not self.ISDETECTING:
                self.ISDETECTING = True
                self.DIR = False
                # print(str(self.rob_name)+"turnAround:" + str(robot.IFMOVE))
                _thread.start_new_thread(self.detectAngle, (init_face_dir,taeget_pos))
            if self.truningangle > rad_robust:
                self.activate_motion = 'turnLeft30'
                self.startMotion(self.turnLeft30)
            else:
                self.activate_motion = 'turnRight30'
                self.startMotion(self.turnRight30)
            init_face_dir = self.inertialUnit.getRollPitchYaw()
            self.truningangle = self.getAngle(taeget_pos, Pos, init_face_dir)
        return

    def turn_label(self, target_pos):  # Ziyuan Liu
        # start a new thread to check and change direction
        self.ISDETECTING = False
        _thread.start_new_thread(self.turnAround, (target_pos,))
        # print(str(self.rob_name)+"turn_label:"+str(robot.IFMOVE))
        return

    def if_catch_ball(self, ballPos):  # Boyu Shi
        # check if robot is close enough to ball
        Pos = self.gps.getValues()
        ball_dis = math.sqrt((ballPos[0] - Pos[0]) ** 2 + (ballPos[1] - Pos[1]) ** 2)
        if ball_dis < 0.3:
            self.CATCH_BALL = True
        else:
            self.CATCH_BALL = False
        return

    """not in use
    def goal_dir(self):#Boyu Shi
        #randmly choose a direction to shoot
        xx = random.uniform(-goal_x_width / 2, goal_x_width / 2)
        selfpos = self.gps.getValues()
        if self.team == 'B':
            target_pos = [xx, -goal_dis, 0]
        elif self.team == 'R':
            target_pos = [xx, goal_dis, 0]
        else:
            print(str(self.rob_name)+"Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        dir = self.getAngle(target_pos, selfpos, self.inertialUnit.getRollPitchYaw())
        return dir
    """

    def ifshoot(self, ballPos):  # Boyu Shi
        # check if the ball is in a suitable position to shoot
        self.IFSHOOT = False
        if self.team == 'R':
            if -goal_x_width / 2 < ballPos[0] < goal_x_width / 2 and goal_dis - 1 <= ballPos[1] <= goal_dis:
                self.IFSHOOT = True
        elif self.team == 'B':
            if -goal_x_width / 2 < ballPos[0] < goal_x_width / 2 and -goal_dis < ballPos[1] <= 1 - goal_dis:
                self.IFSHOOT = True
        else:
            print(
                str(self.rob_name) + "Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        return

    def isback(self, player_pos, ball_pos):  # Boyu Shi
        # check if robot,ball and goal are orderly in a line
        middle_line = 2.5
        turn_dir = -1  # -1 isback 0 targe_tpoint is at the back of the ball 1 -x 2 +x 3 down/up
        if self.team == 'R':
            if ball_pos[1] <= -middle_line:
                if player_pos[1] < ball_pos[1]:
                    turn_dir = -1
                else:
                    if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                        turn_dir = 1
                    elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                        turn_dir = 2
                    else:
                        turn_dir = 3
            elif -middle_line < ball_pos[1] < middle_line:
                if player_pos[1] < ball_pos[1]:
                    if ball_pos[0] == player_pos[0]:
                        k3 = 0
                    else:
                        k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                    b3 = ball_pos[1] - k3 * ball_pos[0]
                    if k3 != 0:
                        x_t = (goal_dis - b3) / k3
                    else:
                        x_t = ball_pos[0]
                    if -bottom_line_width / 2 < x_t < bottom_line_width / 2:
                        turn_dir = -1
                    elif x_t<-bottom_line_width/2:
                        turn_dir = 2
                    elif x_t>bottom_line_width/2:
                        turn_dir = 1

                else:
                    if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                        turn_dir = 1
                    elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                        turn_dir = 2
                    else:
                        turn_dir = 3

            elif ball_pos[1] >= middle_line:
                if player_pos[1] < ball_pos[1]:
                    if ball_pos[0] == player_pos[0]:
                        k3 = 0
                    else:
                        k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                    b3 = ball_pos[1] - k3 * ball_pos[0]
                    if k3 != 0:
                        x_t = (goal_dis - b3) / k3
                    else:
                        x_t = ball_pos[0]
                    if -goal_x_width / 2 < x_t < goal_x_width / 2:
                        turn_dir = -1
                    elif x_t < -bottom_line_width / 2:
                        turn_dir = 2
                    else:
                        turn_dir = 1
                else:
                    if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                        turn_dir = 1
                    elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                        turn_dir = 2
                    else:
                        turn_dir = 3
            elif self.team == 'B':
                if ball_pos[1] >= middle_line:
                    if player_pos[1] > ball_pos[1]:
                        self.ISBACK = True
                    else:
                        if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                            turn_dir = 1
                        elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                            turn_dir = 2
                        else:
                            turn_dir = 3
                elif -middle_line < ball_pos[1] < middle_line:
                    if player_pos[1] > ball_pos[1]:
                        if ball_pos[0] == player_pos[0]:
                            k3 = 0
                        else:
                            k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                        b3 = ball_pos[1] - k3 * ball_pos[0]
                        if k3 != 0:
                            x_t = (goal_dis - b3) / k3
                        else:
                            x_t = ball_pos[0]
                        if -bottom_line_width / 2 < x_t < bottom_line_width / 2:
                            self.ISBACK = True
                        elif x_t < -bottom_line_width / 2:
                            turn_dir = 1
                        elif x_t > bottom_line_width / 2:
                            turn_dir = 2
                    else:
                        if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                            turn_dir = 1
                        elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                            turn_dir = 2
                        else:
                            turn_dir = 3

                elif ball_pos[1] <= -middle_line:
                    if player_pos[1] > ball_pos[1]:
                        if ball_pos[0] == player_pos[0]:
                            k3 = 0
                        else:
                            k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                        b3 = ball_pos[1] - k3 * ball_pos[0]
                        if k3 != 0:
                            x_t = (goal_dis - b3) / k3
                        else:
                            x_t = ball_pos[0]
                        if -goal_x_width / 2 < x_t < goal_x_width / 2:
                            self.ISBACK = True
                        elif x_t < -bottom_line_width / 2:
                            turn_dir = 1
                        elif x_t > bottom_line_width / 2:
                            turn_dir = 2
                    else:
                        if ball_pos[0] - 0.5 < player_pos[0] <= ball_pos[0]:
                            turn_dir = 1
                        elif ball_pos[0] < player_pos[0] < ball_pos[0] + 0.5:
                            turn_dir = 2
                        else:
                            turn_dir = 3
        return turn_dir

    def if_change_target(self, previous_pos, ball_pos):#Boyu Shi
        #If the robot's original target position is closer to itself in the direction of attack,
        # then change the target point, as it no longer needs to go around the back
        self.CHANGE_TARGET = False
        if self.team == 'B':
            if previous_pos[1] > ball_pos[1]:
                self.CHANGE_TARGET = True
        elif self.team == 'R':
            if previous_pos[1] < ball_pos[1]:
                self.CHANGE_TARGET = True
        else:
            print(
                str(self.rob_name) + "Uncorrectly name the player:" + self.rob_name + "You need to set team name R or B first")
            return -1
        #
        if math.sqrt((ball_pos[0] - previous_pos[0]) ** 2 + (ball_pos[1] - previous_pos[1]) ** 2)>0.5:
            self.CHANGE_TARGET = True
        Pos = robot.gps.getValues()
        if math.sqrt((Pos[0] - previous_pos[0]) ** 2 + (Pos[1] - previous_pos[1]) ** 2) < 0.2:
            self.CHANGE_TARGET = True
        return self.CHANGE_TARGET

    def point_to_line_distance(self, point, line_point1, line_point2):  # Liu Ziyuan
        if line_point1 == line_point2:
            point_array = np.array(point)
            point1_array = np.array(line_point1)
            distance = np.linalg.norm(point_array - point1_array)
        else:
            A = line_point2[1] - line_point1[1]
            B = line_point1[0] - line_point2[0]
            C = (line_point1[1] - line_point2[1]) * line_point1[0] + (line_point2[0] - line_point1[0]) * line_point1[1]
            distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A ** 2 + B ** 2))
        return distance

    def point_line_side(self, point, line_point1, line_point2):
        tmp = (line_point1[1] - line_point2[1]) * point[0] + (line_point2[0] - line_point1[0]) * point[1] + line_point1[
            0] * line_point2[1] - line_point2[0] * line_point1[1]
        return tmp

    def if_obstacle(self):
        self_pos = self.gps.getValues()
        target_pos = self.target_pos
        self.obstacle = np.zeros(8)
        if target_pos[0] == self_pos[0]:
            s = [self_pos[0] + 2, self_pos[1]]
        else:
            k_sb = (target_pos[1] - self_pos[1]) / (target_pos[0] - self_pos[0])
            if k_sb == 0:
                s = [self_pos[0], self_pos[1] + 2]
            else:
                k_tangent = -1 / k_sb
                s = np.zeros(2)
                s[0] = self_pos[0] + 2
                s[1] = self_pos[1] + (k_tangent * 2)
        closest_dis = -1
        closest_player = 0
        for i in range(Robot_num * 2):
            if robotlist[i] != self.rob_name:
                if (self.point_line_side(target_pos, self_pos, s) * self.point_line_side(self.newest[i], self_pos,
                                                                                         s)) > 0:
                    distance = self.point_to_line_distance(self.newest[i], self_pos, target_pos)
                    point_point_dis = math.sqrt(
                        pow(self_pos[0] - self.newest[i][0], 2) + pow(self_pos[1] - self.newest[i][1], 2))
                    if distance <= R_robot:
                        self.obstacle[i] = 1
                        if (closest_dis == -1 or point_point_dis < closest_dis):
                            closest_dis = point_point_dis
                            closest_player = i
        if closest_dis != -1:
            target_dis = np.sqrt((self_pos[0] - target_pos[0]) ** 2 + (self_pos[1] - target_pos[1]) ** 2)
            if target_dis < closest_dis:
                closest_dis = -1

        return closest_player, closest_dis

    def shortest_tangent(self, obstacle):  # Liu Ziyuan
        Pos = self.gps.getValues()
        obstacle[0] = obstacle[0] - Pos[0]
        obstacle[1] = obstacle[1] - Pos[1]
        if (math.sqrt(pow(obstacle[0], 2) + pow(obstacle[1], 2))) <= R_robot:
            return Pos
        else:
            K_ra = ((obstacle[0] * obstacle[1]) + (
                    R_robot * math.sqrt(pow(obstacle[0], 2) + pow(obstacle[1], 2) - pow(R_robot, 2)))) / (
                           pow(obstacle[0], 2) - pow(R_robot, 2))
            K_rb = ((obstacle[0] * obstacle[1]) - (
                    R_robot * math.sqrt(pow(obstacle[0], 2) + pow(obstacle[1], 2) - pow(R_robot, 2)))) / (
                           pow(obstacle[0], 2) - pow(R_robot, 2))
            tangent_A = [(obstacle[0] + K_ra * obstacle[1]) / 1 + pow(K_ra, 2),
                         ((obstacle[0] + K_ra * obstacle[1]) / 1 + pow(K_ra, 2) * K_ra)]
            tangent_B = [(obstacle[0] + K_rb * obstacle[1]) / 1 + pow(K_rb, 2),
                         ((obstacle[0] + K_rb * obstacle[1]) / 1 + pow(K_rb, 2) * K_rb)]
            delta_OAx = tangent_A[0] - Pos[0]
            delta_OAy = tangent_A[1] - Pos[1]
            dis_OA = math.sqrt(pow(delta_OAx, 2) + pow(delta_OAy, 2))
            delta_OBx = tangent_B[0] - Pos[0]
            delta_OBy = tangent_B[1] - Pos[1]
            dis_OB = math.sqrt(pow(delta_OBx, 2) + pow(delta_OBy, 2))
            if dis_OA > dis_OB:
                return tangent_B
            else:
                return tangent_A

robot = Nao()

robotlist = []


def initial_robotlist():  # Boyu Shi
    # list name of robots
    for i in range(Robot_num):
        robotlist.append("R" + str(i))
        robotlist.append("B" + str(i))


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
initial_robotlist()
# Main loop:
# - perform simulation steps until Webots is stopping the controller

# main written by Boyu Shi
robot.target_pos = robot.gps.getValues()
robot.stand.play()

while robot.step(timestep) != -1:
    if (robot.receive_message()):
        ball_pos = robot.newest[8]
        Pos = robot.gps.getValues()

        if(Pos[2]<0.2):
            robot.activate_motion='standup'
            robot.startMotion(robot.standup)

        if (robot.activate_motion != None):
            present_time = robot.getTime()
            delta_time = present_time - robot.motion_start_time
            if (not delta_time >= robot.motion_time_direc[robot.activate_motion]):
                continue
            else:
                robot.currentlyPlaying.stop()
                robot.activate_motion = None
        if (robot.target_is == -1):
            print("set target ballpos")
            robot.target_is = 0
            robot.target_pos = ball_pos
        if (robot.if_change_target(robot.target_pos, ball_pos)):
            print("change target\n")
            ib = robot.isback(Pos, ball_pos)
            print("ib:"+str(ib)+"\n")
            if (ib==-1):
                robot.target_is = 0
                print(str(robot.rob_name) + 'target pos is ballpos\n')
                robot.target_pos = ball_pos
            else:
                robot.target_is = 1
                if (ib == 0):
                    print(str(robot.rob_name) + 'target pos is back ballpos\n')
                    if (robot.team == "R"):
                        if (Pos[1] > ball_pos[1] - 0.2):
                            robot.target_pos = [Pos[0], Pos[1] - 0.2]
                        else:
                            robot.target_pos = [ball_pos[0], ball_pos[1] - 0.1]
                    elif robot.team == "B":
                        if (Pos[1] < ball_pos[1] + 0.2):
                            robot.target_pos = [Pos[0], Pos[1] + 0.2]
                        else:
                            robot.target_pos = [ball_pos[0], ball_pos[1] + 0.1]
                elif ib == 1:
                    print(str(robot.rob_name) + 'target pos is left pos\n')
                    robot.target_pos = [max(Pos[0] - 0.1, -bottom_line_width / 2), Pos[1]]
                elif (ib == 2):
                    print(str(robot.rob_name) + 'target pos is right pos\n')
                    robot.target_pos = [min(Pos[0] + 0.1, bottom_line_width / 2), Pos[1]]
                elif (ib == 3):
                    print(str(robot.rob_name) + 'target pos is back pos\n')
                    if (robot.team == "R"):
                        robot.target_pos = [Pos[0], Pos[1] - 1]
                    elif (robot.team == "B"):
                        robot.target_pos = [Pos[0], Pos[1] + 1]
        print(str(robot.rob_name) + ":" + str(robot.target_pos) + '\n')
        player, dis = robot.if_obstacle()
        if (dis != -1):
            robot.target_pos = robot.shortest_tangent(robot.newest[player])
            print(str(robot.rob_name) + 'target pos change to avoid\n' + str(robot.target_pos))
            robot.if_catch_ball(robot.newest[Robot_num * 2])
            """
            if (robot.CATCH_BALL):
                if (robot.target_pos[0] < robot.gps.getValues()[0]):
                    if (robot.team == "R"):
                        robot.activate_motion = "SideStepRight"
                        robot.startMotion(robot.SideStepRight)
                    else:
                        robot.activate_motion = "SideStepLeft"
                        robot.startMotion(robot.SideStepLeft)
                elif (robot.target_pos[0] > robot.gps.getValues()[0]):
                    if (robot.team == "B"):
                        robot.activate_motion = "SideStepRight"
                        robot.startMotion(robot.SideStepRight)
                    else:
                        robot.activate_motion = "SideStepLeft"
                        robot.startMotion(robot.SideStepLeft)
            """
            robot.activate_motion="shoot"
            robot.startMotion(robot.shoot)
        present_face_dir = robot.inertialUnit.getRollPitchYaw()
        robot.if_dir(robot.target_pos, Pos, present_face_dir)
        if (robot.DIR):
            print("face to the target\n")
            robot.if_catch_ball(robot.target_pos)
            if (robot.target_is == 0):
                if (robot.CATCH_BALL):
                    robot.target_is = -1
                    robot.ifshoot(ball_pos)
                    print(str(robot.rob_name) +"catch ball\n")
                    if (robot.IFSHOOT):
                        print(str(robot.rob_name) + "shoot\n")
                        robot.activate_motion = "shoot"
                        robot.startMotion(robot.shoot)
                    else:
                        print(str(robot.rob_name) + "bring ball forward\n")
                        robot.activate_motion = "move"
                        robot.startMotion(robot.move)
                else:
                    print(str(robot.rob_name) + "move to ball\n")
                    robot.activate_motion = "move"
                    robot.startMotion(robot.move)
            elif (robot.target_is == 1):
                print(str(robot.rob_name) + "reach target\n")
                print(robot.CATCH_BALL)
                if (robot.CATCH_BALL):
                    print("reset target pos")
                    robot.target_is = -1
                else:
                    print(str(robot.rob_name) + "move to target\n")
                    robot.activate_motion = "move"
                    robot.startMotion(robot.move)
        else:
            print("turn to target\n")
            robot.turn_label(robot.target_pos)

"""
while robot.step(timestep) != -1:
    if (robot.receive_message()):
        robot.turn_label(robot.newest[Robot_num*2])
"""
