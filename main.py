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

R_robot=0.3

goal_x_width = 2.6
goal_y_width = 1
goal_dis = 4.5
bottom_line_width=6

rad_robust=0.5

class Nao(Robot):
    # load motion files
    def loadMotionFiles(self):
        self.move = Motion('../../motions/Move.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft30 = Motion('../../motions/TurnLeft30.motion')
        self.turnRight30 = Motion('../../motions/TurnRight30.motion')
        self.shoot = Motion('../../motions/Shoot.motion')
        self.stand = Motion('../../motions/Stand.motion')
        self.standup = Motion('../../motions/StandUpFromFront.motion')

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
        self.obstacle = np.zeros(Robot_num * 2)

    def getAngle(self, target_Pos, Pos, RollPitchYaw):#Ziyuan Liu
        #get the angle between robot's face direction and ball's postion
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

    def if_dir(self,target_Pos, Pos, RollPitchYaw):#Ziyuan Liu,Boyu Shi
        #check if robot is directly face to the target position
        self.DIR=False
        delta_x = target_Pos[0] - Pos[0]
        delta_y = target_Pos[1] - Pos[1]
        if delta_x==0 and delta_y==0:
            sin=1
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

    def isback(self, player_pos,ball_pos):#Boyu Shi
        #check if robot,ball and goal are orderly in a line
        middle_line=2.5
        self.ISBACK=False
        turn_dir = -1 #-1 isback 0 targe_tpoint is at the back of the ball 1 inside 2 outside
        if self.team=='R':
            if ball_pos[1]<=-middle_line:
                if player_pos[1]<ball_pos[1]:
                    self.ISBACK=True
            elif -middle_line<ball_pos[1]<middle_line:
                k1=(goal_dis-ball_pos[1])/(bottom_line_width/2-ball_pos[0])
                b1=ball_pos[1]-k1*ball_pos[0]
                k2=(goal_dis-ball_pos[1])/(-bottom_line_width/2-ball_pos[0])
                b2=ball_pos[1]-k2*ball_pos[0]
                k3=(ball_pos[1]-player_pos[1])/(ball_pos[0]-player_pos[0])
                b3=ball_pos[1]-k3*ball_pos[0]
                if -bottom_line_width/2<(goal_dis-b3)/k3<bottom_line_width/2 and player_pos[1]<ball_pos[1]:
                    self.ISBACK=True
                elif k1*player_pos[0]+b1<player_pos[1] and k2*player_pos[0]+b2<player_pos[1]:
                    turn_dir=random.randint(1,2)
                else:
                    turn_dir=0
            elif ball_pos[1]>=middle_line:
                k1=(goal_dis-ball_pos[1])/(goal_x_width/2-ball_pos[0])
                b1=ball_pos[1]-k1*ball_pos[0]
                k2=(goal_dis-ball_pos[1])/(-goal_x_width/2-ball_pos[0])
                b2=ball_pos[1]-k2*ball_pos[0]
                k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                b3 = ball_pos[1] - k3 * ball_pos[0]
                if -goal_x_width/2<(goal_dis-b3)/k3<goal_x_width/2 and player_pos[1]<ball_pos[1]:
                    self.ISBACK=True
                elif k1*player_pos[0]+b1<player_pos[1] and k2*player_pos[0]+b2<player_pos[1]:
                    turn_dir=random.randint(1,2)
                else:
                    turn_dir=0
            elif self.team=='B':
                if ball_pos[1] >= middle_line:
                    if player_pos[1] > ball_pos[1]:
                        self.ISBACK = True
                elif -middle_line < ball_pos[1] < middle_line:
                    k1 = (-goal_dis - ball_pos[1]) / (bottom_line_width / 2 - ball_pos[0])
                    b1 = ball_pos[1] - k1 * ball_pos[0]
                    k2 = (-goal_dis - ball_pos[1]) / (-bottom_line_width / 2 - ball_pos[0])
                    b2 = ball_pos[1] - k2 * ball_pos[0]
                    k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                    b3 = ball_pos[1] - k3 * ball_pos[0]
                    if -bottom_line_width / 2 < (goal_dis - b3) / k3 < bottom_line_width / 2 and player_pos[1] > ball_pos[1]:
                        self.ISBACK = True
                    elif k1 * player_pos[0] + b1 > player_pos[1] and k2 * player_pos[0] + b2 > player_pos[1]:
                        turn_dir = random.randint(1, 2)
                    else:
                        turn_dir = 0
                elif ball_pos[1] <= -middle_line:
                    k1 = (-goal_dis - ball_pos[1]) / (goal_x_width / 2 - ball_pos[0])
                    b1 = ball_pos[1] - k1 * ball_pos[0]
                    k2 = (-goal_dis - ball_pos[1]) / (-goal_x_width / 2 - ball_pos[0])
                    b2 = ball_pos[1] - k2 * ball_pos[0]
                    k3 = (ball_pos[1] - player_pos[1]) / (ball_pos[0] - player_pos[0])
                    b3 = ball_pos[1] - k3 * ball_pos[0]
                    if -goal_x_width / 2 < (goal_dis - b3) / k3 < goal_x_width / 2 and player_pos[1] > ball_pos[1]:
                        self.ISBACK = True
                    elif k1 * player_pos[0] + b1 > player_pos[1] and k2 * player_pos[0] + b2 > player_pos[1]:
                        turn_dir = random.randint(1, 2)
                    else:
                        turn_dir = 0
        return turn_dir

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

    def point_to_line_distance(self,point, line_point1, line_point2):#Liu Ziyuan
        # calculate the distance
        vec1 = [line_point1[0] - point[0],line_point1[1] - point[1]]
        vec1=np.array(vec1)
        vec2 = [line_point2[0] - point[0],line_point2[1] - point[1]]
        vec2 = np.array(vec2)
        distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(np.array(line_point1[:2]) - np.array(line_point2[:2]))
        return distance

    def point_line_side(self, point, line_point1, line_point2):
        tmp = (line_point1[1] - line_point2[1]) * point[0] + (line_point2[0] - line_point1[0]) * point[1] + line_point1[0] * line_point2[1] - line_point2[0] * line_point1[1]
        return tmp

    def if_obstacle(self):
        self_pos = self.gps.getValues()
        target_pos = self.target_pos
        self.obstacle = np.zeros(8)
        k_sb = (target_pos[1] - self_pos[1]) / (target_pos[0] - self_pos[0])
        k_tangent = -1 / k_sb
        s = np.zeros(2)
        closest_dis = -1
        closest_player = 0
        s[0] = self_pos[0] + (2)
        s[1] = self_pos[1] + (k_tangent * 2)
        for i in range(Robot_num*2):
            if (self.point_line_side(target_pos, self_pos, s) * self.point_line_side(self.newest[i], self_pos, s)) > 0:
                distance = self.point_to_line_distance(self.newest[i], self_pos, target_pos)
                point_point_dis = math.sqrt(
                    pow(self_pos[0] - self.newest[i][0], 2) + pow(self_pos[1] - self.newest[i][1], 2))
                if distance <= R_robot:
                    self.obstacle[i] = 1
                    if (closest_dis == -1 or point_point_dis < closest_dis):
                        closest_dis = point_point_dis
                        closest_player = i
        return closest_player,closest_dis

    def shortest_tangent(self, obstacle):#Liu Ziyuan
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
    """cause jamming problem so don't use
    def turn_certain_angle(self,angle):#Boyu Shi
        #input angle in degree,+ ->anticlockwise(turnleft) - ->clockwise(turnright)
        self.TURNING=True
        init_face_dir=self.inertialUnit.getRollPitchYaw()
        angle=angle*math.pi/180
        unit_turn=60*math.pi/180
        if(angle>0):
            self.startMotion(self.turnLeft30)
        else:
            self.startMotion(self.turnRight30)
        while(TURNING):
            present_face_dir = self.inertialUnit.getRollPitchYaw()
            temp = abs(present_face_dir[2] - init_face_dir[2])
            if temp>=abs(angle):
                TURNING=False
                if self.currentlyPlaying:
                    self.currentlyPlaying.stop()
            if temp>=(unit_turn-0.1):
                if(angle>0):
                    angle=angle-unit_turn
                    self.startMotion(self.turnLeft30)
                    if(angle<0):
                        angle=0
                elif(angle<0):
                    angle=angle+unit_turn
                    self.startMotion(self.turnRight30)
                    if(angle>0):
                        angle=0
                if(angle==0):
                    TURNING = False
                    if self.currentlyPlaying:
                        self.currentlyPlaying.stop()
        return

    def move_certain_dis(self,dis):#Boyu Shi
        move_ok=False
        initpos=self.gps.getValues()
        print(str(self.rob_name)+"start move")
        while(not move_ok):
            self.startMotion(self.move)
            Pos = self.gps.getValues()
            move_dis=math.sqrt((Pos[0]-initpos[0])**2+(Pos[1]-initpos[1])**2)
            print(str(self.rob_name)+"already move:"+str(move_dis))
            if move_dis>=dis:
                move_ok=True
        return

    
    def circle_turn(self, turn_dir,ballpos):#Boyu Shi
        #Turn around the ball
        self.startMotion(self.backwards)
        print(str(self.rob_name)+"start backwards")
        while(self.CATCH_BALL):
            self.if_catch_ball(ballpos)
        if turn_dir == 'r':
            self.turn_certain_angle(-60)
            self.move_certain_dis(0.2)
        elif turn_dir == 'l':
            self.turn_certain_angle(60)
            self.move_certain_dis(0.2)
        else:
            print(str(self.rob_name)+"wrong input for circle turn")
        return
    """
    """
    def circle_turn(self, turn_dir):#Boyu Shi
        if turn_dir == 'r':
            self.startMotion(self.sideStepRight)
        elif turn_dir == 'l':
            self.startMotion(self.sideStepLeft)
        else:
            print(str(self.rob_name)+"wrong input for circle turn")
        return
    """

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
        if(Pos[2]<0.2):
            robot.activate_motion='standup'
            robot.startMotion(robot.standup)
        if(robot.activate_motion!=None):
            present_time=robot.getTime()
            delta_time=present_time-robot.motion_start_time
            if(not delta_time>=robot.motion_time_direc[robot.activate_motion]):
               continue
            else:
                robot.currentlyPlaying.stop()
                robot.activate_motion=None
        if(robot.target_is==-1):
            robot.target_is=0
            robot.target_pos=ball_pos
        if(robot.if_change_target(robot.target_pos,ball_pos)):
            ib=robot.isback(Pos,ball_pos)
            if(robot.ISBACK):
                robot.target_is=0
                print(str(robot.rob_name)+'target pos is ballpos\n')
                robot.target_pos=ball_pos
            else:
                robot.target_is=1
                if(ib==0):
                    print(str(robot.rob_name)+'target pos is back ballpos\n')
                    if(robot.team=="R"):
                        if(Pos[1]>ball_pos[1]-0.1):
                            robot.target_pos=[Pos[0],Pos[1]-0.4]
                        else:
                            robot.target_pos=[ball_pos[0], ball_pos[1]-0.4]
                    elif(robot.team=="B"):
                        if (Pos[1] < ball_pos[1] + 0.2):
                            robot.target_pos=[Pos[0],Pos[1]+0.4]
                        else:
                            robot.target_pos = [ball_pos[0], ball_pos[1]+0.4]
                elif(ib==1):
                    print(str(robot.rob_name)+'target pos is left pos\n')
                    robot.target_pos=[max(Pos[0]-0.5,-bottom_line_width/2),Pos[1]]
                elif(ib==2):
                    print(str(robot.rob_name)+'target pos is right pos\n')
                    robot.target_pos=[min(Pos[0]+0.5,bottom_line_width/2),Pos[1]]

        player,dis=robot.if_obstacle()
        if(dis!=-1):
            robot.target_pos=robot.shortest_tangent(robot.newest[player])
            print(str(robot.rob_name) + 'target pos change to avoid\n')
            robot.if_catch_ball(robot.newest[Robot_num*2])
            if(robot.CATCH_BALL):
                if(robot.target_pos[0]<robot.gps.getValues()[0]):
                    if(robot.team=="R"):
                        robot.activate_motion = "sideStepRight"
                        robot.startMotion(robot.SideStepRight)
                    else:
                        robot.activate_motion = "sideStepLeft"
                        robot.startMotion(robot.SideStepLeft)
                elif(robot.target_pos[0]>robot.gps.getValues()[0]):
                    if (robot.team == "B"):
                        robot.activate_motion = "sideStepRight"
                        robot.startMotion(robot.SideStepRight)
                    else:
                        robot.activate_motion = "sideStepLeft"
                        robot.startMotion(robot.SideStepLeft)
        present_face_dir = robot.inertialUnit.getRollPitchYaw()
        robot.if_dir(robot.target_pos, Pos, present_face_dir)
        if(robot.DIR):
            robot.if_catch_ball(robot.target_pos)
            if(robot.target_is==0):
                if(robot.CATCH_BALL):
                    robot.target_is = -1
                    robot.ifshoot(ball_pos)
                    if(robot.IFSHOOT):
                        robot.activate_motion = "shoot"
                        robot.startMotion(robot.shoot)
                    else:
                        robot.activate_motion = "move"
                        robot.startMotion(robot.move)
                else:
                    robot.IFSHOOT=False
                    robot.activate_motion = "move"
                    robot.startMotion(robot.move)
            elif(robot.target_is==1):
                if (robot.CATCH_BALL):
                    robot.target_is=-1
                else:
                    robot.activate_motion = "move"
                    robot.startMotion(robot.move)
        else:
            robot.turn_label(robot.target_pos)

                    

"""
while robot.step(timestep) != -1:
    robot.startMotion(robot.shoot)
"""
