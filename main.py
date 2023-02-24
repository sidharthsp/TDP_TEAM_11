"""main controller."""
#Author:Boyu Shi
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

Robot_num=8

class Nao (Robot):

    # load motion files
    def loadMotionFiles(self):
        self.move = Motion('../../motions/Move.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.shoot = Motion('../../motions/Shoot.motion')

    def startMotion(self, motion):
        # interrupt current motion
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

        #receiver
        receiver_name=self.getName()+"r"
        self.receiver=self.getDevice(receiver_name)

        #GPS
        self.gps = self.getDevice(self.getName()+"gps")
        self.gps.enable(self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice(self.getName()+"IU")
        self.inertialUnit.enable(self.timeStep)

    def receive_meaasge(self):
        flag=0
        if(self.receiver.getQueueLength()>0):
            message=self.receiver.getFloats()
            length=len(message)
            print(message)
            epoch_size=(Robot_num+1)*3
            epoch=length/epoch_size
            self.newest=[]
            for i in range(int((epoch-1)*epoch_size),int(epoch*epoch_size),3):
                pos=[message[i],message[i+1],message[i+2]]
                if(self.first_pos==0):
                    self.newest.append(pos)
                else:
                    self.newest[int(i/3)]=pos
            flag=1
            self.receiver.nextPacket()
        return flag

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        # initialize
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.receiver.enable(20)
        self.first_pos=0
        self.ZERO_ANGLE = 0
        self.catch_ball=False

    def getAngle(self, ballPos, Pos, RollPitchYaw):
        delta_x = ballPos[0] - Pos[0]
        delta_y = ballPos[1] - Pos[1]
        sin = delta_y / math.sqrt(pow(delta_x,2) + pow(delta_y,2))
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
            Angle += 2*math.pi
        if Angle > math.pi:
            Angle -= 2*math.pi

        return Angle

    def detectAngle(self, init_face_dir, angle):
        while True:
            present_face_dir = self.inertialUnit.getRollPitchYaw()
            temp = abs(present_face_dir[2] - init_face_dir[2] - angle)
            if temp < 0.2:
                if self.currentlyPlaying:
                    self.currentlyPlaying.stop()
                    self.currentlyPlaying = None
                self.ISDETECTING = False
                return

    def turnAround(self,ballPos):
        Pos = self.gps.getValues()
        init_face_dir = self.inertialUnit.getRollPitchYaw()
        angle = self.getAngle(ballPos, Pos, init_face_dir)
        if abs(angle) >= 0.2:
            if not self.ISDETECTING:
                self.ISDETECTING = True
                _thread.start_new_thread(self.detectAngle,(init_face_dir, angle))
            if angle > 0:
                self.startMotion(self.turnLeft60)
            else:
                self.startMotion(self.turnRight60)

    def turn_label(self,ballPos):
        self.ISDETECTING = False
        _thread.start_new_thread(self.turnAround, (ballPos,))

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
    #get the position of ball(and other robots)
    if(robot.receive_meaasge()):
        ball_pos=robot.newest[8]
        print(ball_pos)
        robot.turn_label(ball_pos)
        #calculate angle and distance

        #decide turn,move or shoot

        #decide where to go next step

        #change angle

    #move forward
    if not robot.currentlyPlaying:
        robot.startMotion(robot.move)


# Enter here exit cleanup code.
