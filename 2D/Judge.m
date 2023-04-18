%% (sidharth) (Adapted from Judge.py )
function [players,ball]  = Judge(players,ball)
    for i = 1:8
        

    %   i = 1;
    dx = ball.Position(1) - players(i).Position(1);
    dy = ball.Position(2) - players(i).Position(2);
    angle = atan2(dy, dx);
    if angle < 0
        angle = angle + 2*pi;
    end


    if players(i).Role == 1 %%Striker
        if strcmp(players(i).team,'red')
            post = [-4.2 0];
        else
            post = [4.2 0];
        end
        distance = norm(players(i).Position - ball.Position); % Calculate the distance between the player and the ball
        goal_dist = norm(players(i).Position - post);

        if distance < 0.2 && goal_dist < 1.9
            dx = post(1) - players(i).Position(1);
            dy = post(2) - players(i).Position(2);
            angle = atan2(dy, dx);
            if angle < 0
                angle = angle + 2*pi;
            end
            players(i).Direction = angle;
        
            ball = players(i).shoot(ball);
        elseif distance < 0.2 % Assume 1 meter is the maximum distance for kicking
            dx = ball.Position(1) - players(i).Position(1);
            dy = ball.Position(2) - players(i).Position(2);
            angle = atan2(dy, dx);
            if angle < 0
                angle = angle + 2*pi;
            end
            players(i).Direction = angle;
            ball = players(i).kick(ball);
            
        else
            players(i).Direction = angle;

    %       
   
            players(i) = players(i).move(0.002,players,i);
 
        end
    end
    
    %----------------------------------------------------------------------------------------------    
    if   players(i).Role == 2 %% Defender 
            
            players(i).Direction = angle;
            players(i) = players(i).move(0.001,players,i);
    end    

    %----------------------------------------------------------------------------------------------    
        if players(i).Role == 3 %% Goalkeeper
            distance = norm(players(i).Position - ball.Position); % Calculate the distance between the player and the ball
        

            players(i).Direction = angle;
            if ball.Position(2) < 1.5 && ball.Position(2)> -1.5 && (distance < 0.2)
                if strcmp(players(i).team,'red')
                    players(i).Direction = 0;
                else
                    players(i).Direction = pi;
                end
                players(i) = players(i).save(0.01);
            elseif ball.Position(2) < 1.5 && ball.Position(2)> -1.5
                players(i) = players(i).save(0.01);
            end
        
        end
        
        
    end  

end