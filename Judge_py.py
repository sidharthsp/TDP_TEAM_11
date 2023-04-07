"""Judge_py controller."""
#Author:Boyu Shi
import math

from controller import Robot
from controller import Emitter
from controller import Supervisor
from controller import Node

Robot_num = 4
x_width=6
y_width=9
goal_x_width = 2.6
goal_y_width = 1
goal_dis = 4.5
initial_pos = [[0, -4.4, 0.31], [0, 4.4, 0.31], [0.27, -0.87, 0.31], [-0.19, 0.39, 0.31],
               [-0.91, -2.27, 0.31], [0.91, 2.27, 0.31], [-1.57, -0.6, 0.31], [1.57, 0.6, 0.31],[0,0,0.0699224]]

# create the Robot instance.
class Judge(Supervisor):
    def set_score(self):
        ball_control="None"
        if(robot.belong_team==1):
            ball_control="Blue"
        elif(robot.belong_team==2):
            ball_control="Red"
        st="Red score:"+str(self.rscore)+"  Blue score:"+str(self.bscore)+"  Ball control: "+ball_control
        self.setLabel( 0, st, 0.3, 0.01, 0.2, 0x000000)

    def check_goal(self):
        ball_pos = self.robo_ball_pos[Robot_num*2]
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
            robot.ballinitial()

    def ballinitial(self):
        ball_trans.setSFVec3f([0, 0, 0.0699224])

    def initial_robotlist(self):
        for i in range(Robot_num):
            self.robotlist.append("R" + str(i))
            self.robotlist.append("B" + str(i))

    def robot_initial(self):
        for i in range(Robot_num * 2):
            node = robot.getFromDef(self.robotlist[i])
            transl = node.getField("translation")
            transl.setSFVec3f(initial_pos[i])

    def sendmessage(self):
        ball_trans = ball.getPosition()
        temp = []
        for i in range(Robot_num * 2):
            node = robot.getFromDef(self.robotlist[i])
            transl = node.getPosition()
            for j in range(3):
                temp.append(transl[j])
            self.robo_ball_pos[math.floor(i/2)]=transl
        for i in range(3):
            temp.append((ball_trans[i]))
        self.robo_ball_pos[Robot_num*2]=ball_trans
        emt.send(temp)

    def changeRobopos(self,rob, pos):
        node = robot.getFromDef(self.robotlist[rob])
        transl = node.getField("translation")
        transl.setSFVec3f(pos)

    def getup(self,rob,pos):
        node = robot.getFromDef(self.robotlist[rob])
        rotation = node.getField("rotation")
        transl = node.getField("translation")
        if rob%2==0:
            transl.setSFVec3f([pos[0],pos[1],0.5])
            rotation.setSFRotation([-0.0941427, 0.0920804, 0.991291, 1.59246])
        elif rob%2==1:
            transl.setSFVec3f([pos[0], pos[1], 0.5])
            rotation.setSFRotation([0.0920514, 0.0916987, -0.991523, 1.61606])
        return

    def check_fall_down(self):
        for i in range(Robot_num*2):
            node = robot.getFromDef(self.robotlist[i])
            transl=node.getPosition()
            if transl[2]<0.1:
                self.getup(i,transl)
        return

    def check_belong(self):
        ball_pos=self.robo_ball_pos[2*Robot_num]
        blue_control=0
        red_control=0
        for i in range(2*Robot_num):
            if(math.sqrt((self.robo_ball_pos[i][0]-ball_pos[0])**2+(self.robo_ball_pos[i][1]-ball_pos[1])**2))<=0.3:
                if(i%2==0):
                    red_control+=1
                else:
                    blue_control+=1
            if(red_control and blue_control):
                break
        if(blue_control!=0 and red_control==0):
            self.belong_team=1
        elif(blue_control==0 and red_control!=0):
            self.belong_team=2
        else:
            self.belong_team=0
        return

    def check_out_of_bounds(self):
        ball_pos = self.robo_ball_pos[Robot_num * 2]
        x, y, z = ball_pos
        flag = 0
        if x<-x_width/2 or x>x_width/2 or y<-y_width/2 or y>y_width/2:
            flag=1
        if flag==1:
            if x<-x_width/2:
                x=x+0.3
            if x>x_width/2:
                x=x-0.3
            if y<-y_width/2:
                y=y+0.3
            if y>y_width/2:
                y=y-0.3
            ball_pos=[x,y,z]
            ball_trans.setSFVec3f(ball_pos)
        self.robo_ball_pos[Robot_num * 2]=ball_pos
        return

    def __init__(self):
        super().__init__()
        self.rscore = 0
        self.bscore = 0
        self.robotlist = []
        self.robo_ball_pos=initial_pos
        self.belong_team=0 #0->no team 1->blue 2->red


robot = Judge()
emt = Emitter(name="emitter")

emt.setChannel(0)

ball = robot.getFromDef("ball")
ball_trans = ball.getField("translation")





# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
robot.ballinitial()
robot.initial_robotlist()
robot.robot_initial()
while robot.step(timestep) != -1:
    robot.check_goal()
    robot.check_out_of_bounds()
    robot.check_belong()
    robot.set_score()
    robot.check_fall_down()
    robot.sendmessage()

# Enter here exit cleanup code.
