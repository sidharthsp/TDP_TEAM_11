function obj = setInitialPose(players,positioning,ball)

for i = 1:8
   players(i).Position = positioning(i,:);
   players(i).plot();
end
obj = players;
ball.plot('w');
end