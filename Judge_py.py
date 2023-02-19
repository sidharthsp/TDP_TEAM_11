"""Judge_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Emitter
from controller import Supervisor
from controller import Node

# create the Robot instance.
class Judge(Supervisor):
    def set_score(self):
        st="red team:"+str(self.rscore)+"  blue team:"+str(self.bscore)
        self.setLabel( 0, st, 0.3, 0.01, 0.2, 0x000000)

    def check_goal(self):
        ball_pos = ball.getPosition()
        x, y, z = ball_pos
        flag = 0
        if goal_x_width / 2 > x > -goal_x_width / 2:
            if -goal_dis > y > -goal_y_width - goal_dis:
                self.rscore += 1
                flag = 1
            elif goal_dis < y < goal_y_width + goal_dis:
                self.bscore += 1
                flag = 1
        if flag == 1:
            ballinitial()

    def __init__(self):
        super().__init__()
        self.rscore = 0
        self.bscore = 0


robot = Judge()
emt = Emitter(name="emitter")

Robot_num = 4
goal_x_width = 2.6
goal_y_width = 1
goal_dis = 4.5
initial_pos = [[0, 0.5, 0.31], [1.25, 0.25, 0.31], [-2, 2, 0.31], [0, 4, 0.31],
               [0, -0.5, 0.31], [-1, -1, 0.31], [1.5, -2, 0.31], [0, -4, 0.31]]

emt.setChannel(0)

ball = robot.getFromDef("ball")
ball_trans = ball.getField("translation")


def ballinitial():
    ball_trans.setSFVec3f([0, 0, 0.0699224])


robotlist = []


def initial_robotlist():
    for i in range(Robot_num):
        robotlist.append("R" + str(i))
        robotlist.append("B" + str(i))


def robot_initial():
    for i in range(Robot_num * 2):
        node = robot.getFromDef(robotlist[i])
        transl = node.getField("translation")
        transl.setSFVec3f(initial_pos[i])


def sendmessage():
    ball_trans = ball.getPosition()
    temp=[]
    for i in range(Robot_num * 2):
        node = robot.getFromDef(robotlist[i])
        transl = node.getPosition()
        for i in range(3):
            temp.append(transl[i])
    for i in range(3):
        temp.append((ball_trans[i]))
    emt.send(temp)


def changeRobopos(rob, pos):
    node = robot.getFromDef(robotlist[rob])
    transl = node.getField("translation")
    transl.setSFVec3f(pos)


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
ballinitial()
initial_robotlist()
#robot_initial()
while robot.step(timestep) != -1:
    robot.check_goal()
    robot.set_score()
    sendmessage()

# Enter here exit cleanup code.
