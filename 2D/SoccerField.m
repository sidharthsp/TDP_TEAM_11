
%% Code by Sidharth
figure('Name','TDP_Team_11','NumberTitle','off');
main = axes;
hold on 
% In this convention, 1 => left , and 2 => right

% We define the measurements of the fields. All measurements are in 
% metres.
outer_length = 11.0;
outer_width  = 8.0;
field_length = 9.0;
field_width = 6.0;
goal_width = 3;
goal_length = 1;
goal_post_width = 2.6;
goal_post_length = 0.6;

 penalty_length = 2;
 penalty_width = 5;
center_circle_radius = 0.75;

% Create the field boundaries
outer_bound = rectangle('Position',[-outer_length/2 -outer_width/2 outer_length outer_width], EdgeColor="w", Parent=main);
field = rectangle('Position',[-field_length/2 -field_width/2 field_length field_width],'EdgeColor',"w",Linewidth=3,Parent=main);

% % Create the goals
goal_1 = rectangle('Position',[-field_length/2 -goal_width/2 goal_length goal_width],'EdgeColor',"w",LineWidth=3,Parent=main);
goal_2 = rectangle('Position',[field_length/2-goal_length -goal_width/2 goal_length goal_width],'EdgeColor',"w",LineWidth=3,Parent=main);
 
% % Create the penalty areas
penalty_1 = rectangle('Position',[-field_length/2+penalty_length/2-goal_length -penalty_width/2 penalty_length penalty_width],EdgeColor="w", LineWidth=3,Parent=main);
penalty_2 = rectangle('Position',[field_length/2-penalty_length/2-goal_length -penalty_width/2 penalty_length penalty_width],'EdgeColor',"w",LineWidth=3,Parent=main);
% half_line = line([0 0],[0 3],Linewidth=3, Parent = main);
% half_line1 = line([0 0],[0 -3],Linewidth=3, Parent = main);
half_line = rectangle('Position', [-0.02, -3, 0.04, 6], 'FaceColor', 'w', 'EdgeColor','w',Parent=main);

% Create the center circle and goal posts
goal_post_1 = rectangle('Position',[-field_length/2-goal_post_length -goal_width/2+0.2 goal_post_length goal_post_width],EdgeColor="w",LineWidth=3,Parent=main);
goal_post_2 = rectangle('Position',[field_length/2 -goal_width/2+0.2 goal_post_length goal_post_width],EdgeColor="w",LineWidth=3,Parent=main);
center_circle = rectangle('Position',[-center_circle_radius -center_circle_radius 2*center_circle_radius 2*center_circle_radius],'Curvature',[1,1],LineWidth=3,EdgeColor="w",Parent=main);


% Set the field color
outer_bound.FaceColor = [0.3  0.642 0.447];

% field.FaceColor = 'green';
centre_point = text(0,0,"-",Color="w",Parent=main);
penalty_point_1 = text(-3,-0.05,"+",Color="w",Parent=main);
penalty_point_2 = text(3,-0.05,"+",Color="w",Parent=main);

% To prevent the image from stretching, we equalize the axis.
 axis equal;



