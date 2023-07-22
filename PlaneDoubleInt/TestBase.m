close all

dt=0.5;
tspan=0;
Y=[];
y=[];
robots=[];
n_robots=6;
prev=n_robots;
colors=colormap();
colors=colors(1:floor(end/n_robots):floor(end/n_robots)*n_robots,:);
colororder(colors())


adj_mat=zeros(n_robots,n_robots);
base_adj_line=zeros(1,n_robots);
base_adj_line(1,2)=1;base_adj_line(1,end)=1;
for index = 1:n_robots
    adj_mat(index,:)=circshift(base_adj_line,[0,index-1]);
end

for index = 1:n_robots
    robot=PlaneDoubleIntObj(index);
    robot.set_connections(adj_mat(index,:).*[1:n_robots]);
    robots=[robots,robot]; %#ok
    prev=index;
end

for i=robots
    disp(i.connections(1:n_robots));
end

for index = 1:n_robots
    robots(index).set_state(randn(1,4).*[1,0,1,0]);
    y=[y,robots(index).get_state()];
end
Y=[Y;y];


t=0;

while t<500
    t=t+dt;
    y=[];
    for index = 1:n_robots
        u=robots(index).get_control(robots);
        robots(index).update(u,dt);
        y=[y,robots(index).get_state()];
    end
    Y=[Y;y];
    tspan=[tspan,t-dt];
    if sum(abs(Y(end,:)-Y(end-1,:)))<0.0001
        %break condition
        break
    end

end

figure(1)
colororder(colors())

nexttile
plot(tspan,Y(:,1:4:end),'LineWidth',2,'MarkerIndices',1:5:length(tspan))
legend
nexttile
plot(tspan,Y(:,3:4:end),'LineWidth',2,'MarkerIndices',1:5:length(tspan))
legend
figure(2)
colororder(colors())

plot(Y(:,1:4:end),Y(:,3:4:end),'LineWidth',2,'MarkerIndices',1:5:length(tspan))
legend





