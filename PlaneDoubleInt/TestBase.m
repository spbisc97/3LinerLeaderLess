close all

dt=0.5;
tspan=0;
Y=[];
y=[];
robots=[];
n_robots=4;
prev=n_robots;

%% plot Settings
colors=colormap();
colors=colors(1:floor(end/n_robots):floor(end/n_robots)*n_robots,:);
colors=colors(1:n_robots,:);%add this to each figure
lw=0.6;%linewidth
ls='--';%linestyle
mk='o';%marker
mi=200;%marker interval

%% create adjacency matrix
adj_mat=zeros(n_robots,n_robots);
base_adj_line=zeros(1,n_robots);
base_adj_line(1,2)=1;base_adj_line(1,end)=1;
for index = 1:n_robots
    adj_mat(index,:)=circshift(base_adj_line,[0,index-1]);
end

%% create delta matrix
distance=1;
delta_mat=zeros(n_robots,2);
for i=1:n_robots
    delta_mat(i,1)=distance*cos(2*pi*i/n_robots);
    delta_mat(i,2)=distance*sin(2*pi*i/n_robots);
end


%% init robots
for index = 1:n_robots
    robot=PlaneDoubleIntObj(index);
    robot.set_delta(delta_mat(index,:));
    robot.set_connections(adj_mat(index,:).*[1:n_robots]);
    robots=[robots,robot]; %#ok
    prev=index;
end
%% set initial robot position
for index = 1:n_robots
    robots(index).set_state(randn(1,4).*[1,0,1,0]);
    y=[y,robots(index).get_state()];
end
Y=[Y;y];
%% show adj matrix from robots
for i=robots
    disp(i.connections(1:n_robots));
end

%% run simulation
t=0;
while t<10000
    t=t+dt;
    y=[];
    for index = 1:n_robots
        u=robots(index).get_controls();
        robots(index).update(u,dt,robots);
        y=[y,robots(index).get_actual_state()];
    end
    Y=[Y;y];
    tspan=[tspan,t-dt];
    if sum(abs(Y(end,:)-Y(end-1,:)))<1e-5
        %break condition
        break
    end

end
%% plots
%time-position plane
figure(1)
colororder(colors())
nexttile
plot(tspan,Y(:,1:4:end),...
    'LineWidth',lw, ...
    'LineStyle',ls,...
    'Marker',mk,...
    'MarkerIndices',1:mi:length(tspan))
legend()
nexttile
plot(tspan,Y(:,3:4:end), ...
    'LineWidth',lw, ...
    'LineStyle',ls,...
    'Marker',mk,...
    'MarkerIndices',1:mi:length(tspan))
legend

%x-y plane
figure(2)
colororder(colors())

plot(Y(:,1:4:end),Y(:,3:4:end), ...
    'LineWidth',lw, ...
    'LineStyle',ls,...
    'Marker',mk,...
    'MarkerIndices',1:mi:length(tspan))
axis square
hold on

plot(sin(tspan/10),cos(tspan/10), ...
    'LineWidth',0.1 )

legend





