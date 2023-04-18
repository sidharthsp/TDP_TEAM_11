function Scores = updateScore(ball,Scores)
%UPDATESCORE Summary of this function goes here
%   Detailed explanation goes here

    if ((ball.Position(1) <= -4.5) && (ball.Position(1) > -5) && (ball.Position(2) <= 1) && (ball.Position(2) > -1) )  
        Scores(2) = Scores(2) + 1;
        str = 'Team Red has scored!!';
        status = text(0, -3.5, sprintf('%s \n\n Ball will be reset to the centre and the game shall resume.',str), 'HorizontalAlignment', 'center', 'FontSize', 14,'Color','r','FontWeight','bold');
        pause(1);
        

    elseif (ball.Position(1) >= 4.5 && ball.Position(1) < 5 && ball.Position(2) <= 1 && ball.Position(2) > -1)
        Scores(1) = Scores(1) + 1;
        str = 'Team Blue has scored!!';
        status = text(0, -3.5, sprintf('%s \n\n Ball will be reset to the centre and the game shall resume.',str), 'HorizontalAlignment', 'center', 'FontSize', 14,'Color','b','FontWeight','bold');
        pause(1);
        

    else
        
        str = 'Ball will be reset to the centre and the game shall resume.';
        status = text(0, -3.5, sprintf('%s',str), 'HorizontalAlignment', 'center', 'FontSize', 12,'Color','yellow','FontWeight','bold');
        pause(2);
        
    end
     delete(status);
end

