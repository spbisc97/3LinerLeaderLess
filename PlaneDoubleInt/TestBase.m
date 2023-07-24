close all

dt=0.25;
tspan=0;
Y=[];
robots=[];
n_robots=10;
prev=n_robots;

%% plot Settings
colors=colormap();
colors=colors(1:floor(end/n_robots):floor(end/n_robots)*n_robots,:);
colors=colors(1:n_robots,:);%add this to each figure
lw=0.2;%linewidth
ls='--';%linestyle
mk='o';%marker
mi=500;%marker interval

%% create adjacency matrix
adj_mat=zeros(n_robots,n_robots);
base_adj_line=zeros(1,n_robots);
base_adj_line(1,2)=1;base_adj_line(1,end)=1;
for index = 1:n_robots
    adj_mat(index,:)=circshift(base_adj_line,[0,index-1]);
end

%% create delta matrix
distance=3;
delta_mat=zeros(n_robots,2);
type=2;
if type==1
    for i=1:n_robots
        delta_mat(i,1)=distance*cos(2*pi*i/n_robots);
        delta_mat(i,2)=distance*sin(2*pi*i/n_robots);
    end
elseif type==2
    angle=8*pi;
    for i=1:n_robots
        delta_mat(i,1)=distance*(i/n_robots)*cos(angle*i/n_robots);
        delta_mat(i,2)=distance*(i/n_robots)*sin(angle*i/n_robots);
    end
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
    robots(index).update([0;0],dt,robots);
    
end
y=[];
for index = 1:n_robots
    y=[y,robots(index).get_actual_state()];
end
Y=[Y;y];
%% show adj matrix from robots
for i=robots
    disp(i.connections(1:n_robots));
end

%% run simulation
t=0;
while t<5e3
    t=t+dt;
    y=[];
    for index = 1:n_robots
        u=robots(index).get_controls();
        robots(index).update(u,dt,robots);
        y=[y,robots(index).get_actual_state()];
    end
    Y=[Y;y];
    tspan=[tspan,t-dt];
    if t>100 && sum(abs(Y(end,:)-Y(end-1,:)))<1e-6
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
axis equal
hold on
z_c=[];
for index = 1:n_robots
    z_c=[z_c,robots(index).get_state()];
end
C=[sum(z_c(:,1:4:end))/n_robots,sum(z_c(:,3:4:end))/n_robots];
disp([(((1:n_robots).')),(z_c(:,1:4:end).'),(z_c(:,3:4:end).')])
last_t=tspan(end);
if type==1
    plot(cos(tspan/last_t)+C(1),sin(tspan/last_t)+C(2), ...
        'LineWidth',0.1 )
    axis square
    axis equal
    
elseif type==2
    plot(distance.*(tspan/last_t).*cos(angle*tspan/last_t)+C(1),...
        distance.*(tspan/last_t).*sin(angle*tspan/last_t)+C(2),...
        'LineWidth',0.1)
    axis square
    axis equal
    
end

legend





