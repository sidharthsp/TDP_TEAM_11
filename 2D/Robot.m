
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
        Role;
        collide;
        radius = 0.4;


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
            obj.collide = false;
           
        end
        
        function collide = collision(obj, other)
            dist = norm(obj.Position - other.Position);
            obj.collide = false;
            if dist < 0.4
                obj.collide = true;
            end
            if obj.Position(1) < -4.5  || obj.Position(1) > 4.5  
                obj.collide = true;
            end
            if obj.Position(2) < -3  || obj.Position(1) > 3 
                obj.collide = true;
            end
            collide = obj.collide;
        end

        function obj =  move(obj, dt,players,j)
            % A method to update the position of the player based on speed and direction
%             if obj.Position(2) < 3 &&  obj.Position(2) > -3 && obj.Position(1) < 4.5 && obj.Position(1) > -4.5 
            position = obj.Position + obj.Speed * dt * [cos(obj.Direction), sin(obj.Direction)];
            obj.collide = false;
            for i = 1:8
                if i ~= j
                    obj.collide = obj.collide || collision(obj,players(i));
                end
            end
            if ~obj.collide
                obj.Position = position;
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
            plot(obj.Position(1), obj.Position(2), 'o', 'MarkerSize', 25, 'MarkerFaceColor', obj.team, 'MarkerEdgeColor','black'); % Plot a blue circle for the player
             % Hold on to add more plots
             plot([obj.Position(1), obj.Position(1) + cos(obj.Direction)* 0.4], [obj.Position(2), obj.Position(2) + sin(obj.Direction)*0.4], '-k'); % Plot a black line for the direction
%              text(obj.Position(1), obj.Position(2),obj.Name,Color="w");
             hold off; % Release the hold
        end
    end
end