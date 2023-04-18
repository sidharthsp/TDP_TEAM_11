classdef Ball
    % A class to represent a soccer ball
    properties
        % Define the properties of the ball
        Position % The position of the ball on the field
        Speed % The speed of the ball
        Direction % The direction of the ball's movement
        color
    end
    
    methods
        % Define the methods of the ball
        function obj = Ball(position, speed, direction)
            % The constructor method to create a new ball object
            obj.Position = position;
            obj.Speed = speed;
            obj.Direction = direction;
        end
        
        function obj = move(obj, dt)
            % A method to update the position of the ball based on speed and direction
            newpos = obj.Position + obj.Speed * dt * [cos(obj.Direction), sin(obj.Direction)];

            if (newpos(2) < 3 &&  newpos(2) > -3)  && (newpos(1) < 4.8 && newpos(1) > -4.8)
                obj.Position = newpos;
                
                obj.Speed = obj.Speed - 0.03 * obj.Speed;

            
            end
            % Add some code to handle collisions with boundaries or goals here
            
        end
        
        function plot(obj,cl)
            % A method to plot the ball on the field as a circle 
            hold on;
            plot(obj.Position(1), obj.Position(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor', cl,'MarkerEdgeColor','k','LineWidth',1.5); % Plot a red circle for the ball
            hold off;
        end
        
    end
    
end
