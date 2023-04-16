
% Robotics Team Design Project 2022-2023, Team 11.
% 2D Implementaion of Robocup Soccer in MATLAB vR2022b
%% File containing the simulation loop. Run this file to start the soccer game.

%clear  workspace and the command window
clc 
clear
% Create a map of the soccer field
SoccerField();


Scores = [0 0]; %  Scores for the teams :: blue - red

% ball created
ball = Ball([0, 0], 0, 0);



%% (Sidharth and Krishna)
R0 = Robot('R0','red', [0, 0], 15, -pi,1);
R1 = Robot('R1','red', [0, 0], 10, -pi,2);
R2 = Robot('R2','red', [0, 0], 8, -pi,2);
R3 = Robot('R3','red', [0, 0], 15, -pi,3);
B0 = Robot('B0', 'blue',[0, 0], 5, 1.5 * pi,1);
B1 = Robot('B1', 'blue',[0, 0], 18, 0,2);
B2 = Robot('B2', 'blue',[0, 0], 11, 0,2);
B3 = Robot('B3', 'blue',[0, 0], 12, 0,3);

positions = [0 1;0.25 1.25;2 -2;4 0;-0.7 0;-0.6 -0.6;-2 1.5;-4 0]; %% (Krishna)
% players array that contains all the Robots
players = [R0 R1 R2 R3 B0 B1 B2 B3];


players = setInitialPose(players,positions,ball); % Initial positions set as pethe coordinates in the positions array



 scoreText = text(0, 3.5, sprintf('Team Blue : %d Team Red : %d', Scores(1), Scores(2)), 'HorizontalAlignment', 'center', 'FontSize', 16,'Color','w','FontWeight','bold');

 % Set the timestep for the simulation.
 timestep = 0.1;
 
 % Set the total simulation time.
 totalTime = 50;
 
pause(2);
% % Loop through the simulation time steps.
 for t = 0:timestep:totalTime
     updatePose(players)
%     R0 = R0.move(0.01);
%     B0 = B0.move(0.01);
      ball.plot('w');
      ball = ball.move(0.002);
%     R0.plot();
%     B0.plot();
%     pause(0.1);
%     disp(t);
%     disp(R0.Position);
      [players,ball] = Judge(players,ball);
      disp(ball);
      pause(0.01);
      disp(t);
      set(scoreText, 'String', sprintf('Team Blue : %d -Team Red %d', Scores(1), Scores(2)));
      delete(findobj(gca,'type','line','-not','Tag','SoccerField'))
 end


% pause(2);
% for i = 1:8
%     disp(players(i).Position)
% end
% Judge(players,ball);
% updatePose(players);

% positions = [0 0.7;0.25 1.25;2 -2;4 0;-0.7 0;-1 -1;-2 1.5;-4 0]
