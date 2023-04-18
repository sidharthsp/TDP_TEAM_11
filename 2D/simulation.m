
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


GAMESTATE = 1; % 0 -> PLAY , 1 -> END
ball = Ball([0, 0], 0, 0);


%% (Sidharth and Krishna)
R0 = Robot('R0','red', [0, 0], 20, -pi,1);
R1 = Robot('R1','red', [0, 0], 7, -pi,2);
R2 = Robot('R2','red', [0, 0], 6, -pi,2);
R3 = Robot('R3','red', [0, 0], 11, -pi,3);
B0 = Robot('B0', 'blue',[0, 0], 10, 0,1);
B1 = Robot('B1', 'blue',[0, 0], 8, 0,2);
B2 = Robot('B2', 'blue',[0, 0], 7, 0,2);
B3 = Robot('B3', 'blue',[0, 0], 1.5, 0,3);

positions = [1 0;0.25 1.25;2 -2;4 0;     -0.7 2;-1 -1;-2 1.5;-3.5 1.3]; %% (Krishna)
% players array that contains all the Robots
players = [R0 R1 R2 R3 B0 B1 B2 B3];


players = setInitialPose(players,positions,ball); % Initial positions set as pethe coordinates in the positions array



 scoreText = text(0, 3.5, sprintf('Team Blue : %d Team Red : %d', Scores(1), Scores(2)), 'HorizontalAlignment', 'center', 'FontSize', 16,'Color','w','FontWeight','bold');
 
 % Set the timestep for the simulation.
 timestep = 0.1;
 
 % Set the total simulation time.
 totalTime = 38;
 t= 0;
pause(3);
% % Loop through the simulation time steps.
while(GAMESTATE ~= 0 )
        % for t = 0:timestep:totalTime
        
        while(t <= totalTime + timestep)
            t = t + timestep;
            updatePose(players);
            ball.plot('w');
            ball = ball.move(0.002);
            time = text(-4, 3.5, sprintf('Time : %.1f : %1.f',t,totalTime), 'HorizontalAlignment', 'center', 'FontSize', 12,'Color','yellow','FontWeight','bold');

            [players,ball] = Judge(players,ball);
            pause(0.01);
            delete(time);
            set(scoreText, 'String', sprintf('Team Blue : %d -Team Red %d', Scores(1), Scores(2)));
            delete(findobj(gca,'type','line','-not','Tag','SoccerField'))

            if checkGoal(ball)
                ball.color = 'w';
                break;
            end
            
            if t >= totalTime - 1e-6
                disp("Game Over");
                GAMESTATE = 0 ;

                
            end
            




        end
        
        if GAMESTATE == 0
            status = text(0, -3.5, sprintf('FULL-TIME!!'), 'HorizontalAlignment', 'center', 'FontSize', 16,'Color','w','FontWeight','bold');
            pause(1);
            break;
        end
        players = setInitialPose(players,positions,ball);
        Scores = updateScore(ball,Scores);
        ball.Position = [0 0];

 



end


% pause(2);
% for i = 1:8
%     disp(players(i).Position)
% end
% Judge(players,ball);
% updatePose(players);

% positions = [0 0.7;0.25 1.25;2 -2;4 0;-0.7 0;-1 -1;-2 1.5;-4 0]
