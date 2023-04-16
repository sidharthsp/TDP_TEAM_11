%% (sidharth)
classdef Robot
    % A class to represent a soccer player
    properties
        % Define the properties of the player
        Name % The name of the player
        team % team of the player (here, the colour)
        Position % The position of the player on the field
        Speed % The speed of the player
        Direction % The direction of the player's movement
        Role % role of the player
    end
    
    methods
        % Define the methods of the player
        function obj = Robot(name,team, position, speed, direction,role)
            % The constructor method to create a new player object
            obj.Name = name;
            obj.team = team;
            obj.Position = position;
            obj.Speed = speed;
            obj.Direction = direction;
            obj.Role = role;
        end
        
        function obj =  move(obj, dt)
            % A method to update the position of the player based on speed and direction
            if obj.Position(2) < 3 &&  obj.Position(2) > -3 && obj.Position(1) < 4.5 && obj.Position(1) > -4.5 
                obj.Position = obj.Position + obj.Speed * dt * [cos(obj.Direction), sin(obj.Direction)];
            end
        end
        
        function obj = turn(obj, angle)
            % A method to change the direction of the player by a given angle
            obj.Direction = obj.Direction + angle;
        end
        
        function ball = kick(obj, ball)
            % A method to kick the ball if it is close enough to the player
%             distance = norm(obj.Position - ball.Position); % Calculate the distance between the player and the ball
%             if distance < 0.5 % Assume 1 meter is the maximum distance for kicking
                ball.Speed = 30; % Assume 10 m/s is the kicking speed
                ball.Direction = obj.Direction; % Assume the ball goes in the same direction as the player
           
            
        end
     
        function ball = shoot(obj, ball)
            % A method to kick the ball if it is close enough to the player
%             distance = norm(obj.Position - ball.Position); % Calculate the distance between the player and the ball
%             if distance < 0.5 % Assume 1 meter is the maximum distance for kicking
                ball.Speed = 70; % Assume 10 m/s is the kicking speed
                ball.Direction = obj.Direction; % Assume the ball goes in the same direction as the player
           
            
        end

        function obj = save(obj,dt) 
            if obj.Position(2) < 3 &&  obj.Position(2) > -3 && obj.Position(1) < 4.5 && obj.Position(1) > -4.5 
                obj.Position(2) = obj.Position(2) + obj.Speed * dt *sin(obj.Direction);
            end

        end

        function plot(obj)
            % A method to plot the player on the field as a circle with a line indicating direction
            hold on;
            plot(obj.Position(1), obj.Position(2), 'o', 'MarkerSize', 40, 'MarkerFaceColor', obj.team, 'MarkerEdgeColor','black'); % Plot a blue circle for the player
             % Hold on to add more plots
             plot([obj.Position(1), obj.Position(1) + cos(obj.Direction)* 0.4], [obj.Position(2), obj.Position(2) + sin(obj.Direction)*0.4], '-k'); % Plot a black line for the direction
%              text(obj.Position(1), obj.Position(2),obj.Name,Color="w");
             hold off; % Release the hold
        end
    end
end