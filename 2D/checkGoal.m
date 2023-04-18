function bool = checkGoal(ball)
%CHECKGOAL Summary of this function goes here
%   Detailed explanation goes here
if (ball.Position(1) <= -4.5 && ball.Position(1) > -5) || (ball.Position(1) >= 4.5 && ball.Position(1) < 5)
    disp("in goal box");
    bool = true;
else
    bool = false;
end

end

