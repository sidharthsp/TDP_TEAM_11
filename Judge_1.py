"""Judge_py controller."""
#Author:Boyu Shi
import math
import numpy as np
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
stand_up_cost=3.7
initial_pos = [[0, -4.4, 0.31], [0, 4.4, 0.31], [0.27, -0.87, 0.31], [-0.19, 0.39, 0.31],
               [-0.91, -2.27, 0.31], [0.91, 2.27, 0.31], [-1.57, -0.6, 0.31], [1.57, 0.6, 0.31],[0,0,0.0699224]]

reset_robopos=[[0, -4.4, 0.31], [0, 4.4, 0.31],[3.1,-1.5,0.31],[3.1,1.5,0.31],
           [3.1,-3,0.31],[3.1,3,0.31],[-3.1,-1.5,0.31],[-3.1,1.5,0.31]]

time_limit=10#minutes,the length of time the match is to be played
# create the Robot instance.
class Judge(Supervisor):
    def set_score(self):#Sidharth Sreeja Prashanth
        ball_control="None"
        if(robot.belong_team==1):
            ball_control="Blue"
        elif(robot.belong_team==2):
            ball_control="Red"
        time=self.getTime()
        st="Red score:"+str(self.rscore)+"  Blue score:"+str(self.bscore)+"  Ball control: "+ball_control+" Time:"+str(round(time,2))
        self.setLabel( 0, st, 0.1, 0.01, 0.15, 0x000000)#id, label, x, y, size, color, transparency, font

    def ballinitial(self):#Ziyuan Liu
        ball = self.getFromDef("ball")
        ball_trans = ball.getField("translation")
        ball_trans.setSFVec3f([0, 0, 0.0699224])
        self.initialize_ball = 0
        return

    def check_goal(self):#Sidharth Sreeja Prashanth
        #check if there is a goal and change the score
        ball_pos = self.robo_ball_pos[Robot_num*2]
        x, y, z = ball_pos
        if goal_x_width / 2 > x > -goal_x_width / 2:
            if -goal_dis > y > -goal_y_width - goal_dis and self.initialize_ball==0:
                self.bscore += 1
                self.initialize_ball = 1
                self.ballinitial()
            elif goal_dis < y < goal_y_width + goal_dis and self.initialize_ball==0:
                self.rscore += 1
                self.initialize_ball = 1
                self.ballinitial()
        return

    def initial_robotlist(self): #Boyu Shi
        for i in range(Robot_num):
            self.robotlist.append("R" + str(i))
            self.robotlist.append("B" + str(i))
        return
    def robot_initial(self):#Boyu Shi
        for i in range(Robot_num * 2):
            node = robot.getFromDef(self.robotlist[i])
            transl = node.getField("translation")
            transl.setSFVec3f(initial_pos[i])
        return

    def sendmessage(self):#Sidharth Sreeja Prashanth
        #send GPS infomation to robots
        ball = self.getFromDef("ball")
        ball_trans = ball.getPosition()
        temp = []
        for i in range(Robot_num * 2):
            node = robot.getFromDef(self.robotlist[i])
            transl = node.getPosition()
            for j in range(3):
                temp.append(transl[j])
            self.robo_ball_pos[i]=transl
        for i in range(3):
            temp.append((ball_trans[i]))
        self.robo_ball_pos[Robot_num*2]=ball_trans
        emt.send(temp)
        return

    def changeRobopos(self,rob, pos):#Krishna Rajendran
        #reset the robot's position if it is out of the bondary too mucn
        node = robot.getFromDef(self.robotlist[rob])
        transl = node.getField("translation")
        transl.setSFVec3f(pos)
        return

    def getup(self,rob,pos):#Arpan Gupta
        #get up from the ground when it fall down
        node = robot.getFromDef(self.robotlist[rob])
        rotation = node.getField("rotation")
        transl = node.getField("translation")
        if rob%2==0:
            transl.setSFVec3f([pos[0],pos[1],0.2])
            rotation.setSFRotation([0.579065, -0.604087, -0.547506, -2.09618])
        elif rob%2==1:
            transl.setSFVec3f([pos[0], pos[1], 0.2])
            rotation.setSFRotation([0.578745, 0.583663, -0.569554, 2.07499])
        self.start_standup[rob] = self.getTime()
        return

    def check_fall_down(self):#Arpan Gupta
        #check if the robot fall down
        ball = self.getFromDef("ball")
        ball_pos = ball.getPosition()
        for i in range(Robot_num*2):
            node = robot.getFromDef(self.robotlist[i])
            transl=node.getPosition()
            time=self.getTime()-self.start_standup[i]
            if i>1:
                if transl[2]<0.2 and time>stand_up_cost:
                    self.getup(i,transl)
            elif i==0:
                if not(-goal_x_width/2<=ball_pos[0]<=goal_x_width/2 and -goal_dis<=ball_pos[1]<=-goal_dis+1) and transl[2]<0.2 and time>stand_up_cost:
                    self.getup(i, transl)
            else:
                if not(-goal_x_width/2<=ball_pos[0]<=goal_x_width/2 and goal_dis-1<=ball_pos[1]<=goal_dis) and transl[2]<0.2 and time>stand_up_cost:
                    self.getup(i,transl)
        return

    def check_belong(self):#Krishna Rajendran
        #check which side control the ball
        ball = self.getFromDef("ball")
        ball_pos = ball.getPosition()
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

    def check_out_of_bounds(self):#Krishna Rajendran
        #check if the ball is out of boundary
        ball = self.getFromDef("ball")
        ball_pos = ball.getPosition()
        ball_trans = ball.getField("translation")
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

    def check_out_of_pitch(self):#Krishna Rajendran
        #check if the robot is out of boundary too much
        for i in range(Robot_num * 2):
            node = robot.getFromDef(self.robotlist[i])
            pos = node.getPosition()
            if pos[0]<-3.2 or pos[0]>3.2 or pos[1]<-5 or pos[1]>5 or pos[2]<0:
                self.changeRobopos(i,reset_robopos[i])
        return


    def __init__(self):
        super().__init__()
        self.rscore = 0
        self.bscore = 0
        self.robotlist = []
        self.robo_ball_pos=initial_pos
        self.belong_team=0 #0->no team 1->blue 2->red
        self.initialize_ball=0#Set 1 after scoring until the ball returns to midpoint and then set 0 to prevent repeated scoring due to delay
        self.start_standup=np.zeros(Robot_num*2)


robot = Judge()
emt = Emitter(name="emitter")

emt.setChannel(0)


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
    time=robot.getTime()
    if(time<60*time_limit):
        robot.sendmessage()
        robot.check_goal()
        robot.check_out_of_bounds()
        robot.check_belong()
        robot.set_score()
        robot.check_fall_down()
        robot.check_out_of_pitch()
    else:
        robot.setLabel( 0, "GAME IS OVER", 0.1, 0.01, 0.15, 0x000000)
# Enter here exit cleanup code.
