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
    
    distance = norm(players(i).Position - ball.Position); % Calculate the distance between the player and the ball
    if distance < 0.2 % Assume 1 meter is the maximum distance for kicking
        ball = players(i).kick(ball);
        
    else
        players(i).Direction = angle;
        players(i) = players(i).move(0.002);
    end
end
%----------------------------------------------------------------------------------------------    
  if   players(i).Role == 2 %% Defender 
        
        players(i).Direction = angle;
        players(i) = players(i).move(0.001);
  end    

%----------------------------------------------------------------------------------------------    
    if players(i).Role == 3 %% Goalkeeper
       players(i).Direction = angle;
            if ball.Position(2) < 1.5 && ball.Position(2)> -1.5
                players(i) = players(i).save(0.01);
            end
       
    end
    
    
end  

end