dt=0.5;
tspan=0;
Y=[];
y=[];
robots=[];
n_robots=6;
prev=n_robots;
for index = 1:6
    robot=LineDoubleIntObj(index);
    if index<6
        next=index+1;
    else
        next=1;
    end
    conn=[prev,next];
    robot.set_connections(conn);
    robots=[robots,robot]; %#ok
    prev=index;
end

for i=robots
    disp(i.connections(1:n_robots));
end
for index = 1:6
    robots(index).set_state(randn(1,2).*[5,0]);
    y=[y,robots(index).get_state()];
end
Y=[Y;y];


t=0;

while t<1000
    t=t+dt;
    y=[];

    for index = 1:6
        u=robots(index).get_control(robots);
        robots(index).update(u,dt);
        y=[y,robots(index).get_state()];

    end
    Y=[Y;y];
    tspan=[tspan,t-dt];

end

plot(tspan,Y(:,1:2:end)')