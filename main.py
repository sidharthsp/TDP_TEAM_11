"""main controller."""

from controller import Robot
from controller import Accelerometer
from controller import Camera
from controller import Motor
from controller import Motion
from controller import Receiver

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
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.receiver.enable(20)
        self.first_pos=0

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
        #calculate angle and distance

        #decide turn,move or shoot

        #decide where to go next step

        #change angle

    #move forward
    robot.startMotion(robot.move)


# Enter here exit cleanup code.
