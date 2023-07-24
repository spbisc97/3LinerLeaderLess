classdef DiffDriveObj < handle
    properties (Access = public)
        actual_x
        actual_y
        
        delta_x
        delta_y
        
        x
        y
        theta
        
        vartheta_x
        vartheta_y
        vartheta_theta
        
        vartheta_dot_x
        vartheta_dot_y
        vartheta_dot_theta
        
        vartheta_ddot_x
        vartheta_ddot_y
        vartheta_ddot_theta
        
        index
        connections=zeros(1,128);
    end
    
    properties (Access = private)
        dot_x
        dot_y
        dot_theta
        
        P=0.001;
        D=1;
        K=0.1;
        
        P_theta=0.0005;
        D_theta=0.1;
        K_theta=0.01;
        
        K_alpha=0.01;
        alpha=@(t) sin(t)*0.1;
    end
    
    methods (Access = public)
        function obj = DiffDriveObj(index)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
            obj.theta = 0;
            
            obj.actual_x = 0;
            obj.actual_y = 0;
            
            obj.vartheta_x=0;
            obj.vartheta_y=0;
            obj.vartheta_theta=0;
            obj.vartheta_dot_x=0;
            obj.vartheta_dot_y=0;
            obj.vartheta_dot_theta=0;
            obj.vartheta_ddot_x=0;
            obj.vartheta_ddot_y=0;
            obj.vartheta_ddot_theta=0;
            obj.index = index;
            
            f=@(t) 3+sin(t)+cos(3*t)-sin(4*t)-cos(5*t)+sin(0.5*t);
            obj.alpha=@(t,theta,x,y,vartheta_x,vartheta_y) obj.K_alpha*f(t)*[-sin(theta),cos(theta)]*[vartheta_x-x;vartheta_y-y];
        end
        
        function obj = reset(obj)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
            obj.theta = 0;
            obj.dot_theta = 0;
            obj.vartheta_x=0;
            obj.vartheta_y=0;
            obj.vartheta_theta=0;
            obj.vartheta_dot_x=0;
            obj.vartheta_dot_y=0;
            obj.vartheta_dot_theta=0;
            obj.vartheta_ddot_x=0;
            obj.vartheta_ddot_y=0;
            obj.vartheta_ddot_theta=0;
        end
        
        function obj = update(obj, u, dt, robots)
            
            summs=get_summs(obj,robots);
            u_x=u(1);
            u_y=u(2);
            u_theta=u(3);
            
            obj.x = obj.x +cos(obj.theta) *obj.dot_x * dt;
            obj.dot_x = obj.dot_x + cos(obj.theta)* u_x*dt ;
            obj.y = obj.y + sin(obj.theta) *obj.dot_y * dt;
            obj.dot_y = obj.dot_y +sin(obj.theta)* u_y *dt;
            obj.theta = obj.theta + obj.dot_theta * dt;
            obj.dot_theta = obj.dot_theta + u_theta * dt;
            
            obj.actual_x = obj.x + obj.delta_x;
            obj.actual_y = obj.y + obj.delta_y;
            
            obj.vartheta_x=obj.vartheta_x+obj.vartheta_dot_x*dt;
            obj.vartheta_y=obj.vartheta_y+obj.vartheta_dot_y*dt;
            obj.vartheta_dot_x=obj.vartheta_dot_x+obj.vartheta_ddot_x*dt;
            obj.vartheta_dot_y=obj.vartheta_dot_y+obj.vartheta_ddot_y*dt;
            
            obj.vartheta_ddot_x=-obj.P*summs(1)-obj.D*obj.vartheta_dot_x-u_x*dt;
            obj.vartheta_ddot_y=-obj.P*summs(2)-obj.D*obj.vartheta_dot_y-u_y*dt;
            
            
            obj.vartheta_theta=obj.vartheta_theta+obj.vartheta_dot_theta*dt;
            obj.vartheta_dot_theta=obj.vartheta_dot_theta+obj.vartheta_ddot_theta*dt;
            obj.vartheta_ddot_theta=-obj.P_theta*summs(3)-obj.D_theta*obj.vartheta_dot_theta-u_theta*dt;
            
        end
        
        
        function obj=set_connections(obj,connections)
            for i=connections
                if i~=0 && mod(i,1) == 0
                    obj.connections(i)=true;
                end
            end
        end
        
        function obj=remove_connections(obj,connections)
            for i=connections
                if mod(i,1) == 0
                    obj.connections(i)=false;
                end
            end
        end
        
        
        function obj = set_state(obj, state)
            obj.actual_x = state(1);
            obj.x = state(1)-obj.delta_x;
            obj.dot_x = state(2);
            obj.actual_y = state(3);
            obj.y = state(3)-obj.delta_y;
            obj.dot_y = state(4);
            obj.theta = state(5);
            obj.dot_theta = state(6);
            set_vartheta(obj,[obj.x;obj.y;obj.theta]);
        end
        
        function obj = set_delta(obj, delta)
            obj.delta_x = delta(1);
            obj.delta_y = delta(2);
        end
        
        
        function state = get_state(obj)
            state(1) = obj.x;
            state(2) = obj.dot_x;
            state(3) = obj.y;
            state(4) = obj.dot_y;
            state(5) = obj.theta;
            state(6) = obj.dot_theta;
        end
        
        function state = get_actual_state(obj)
            state(1) = obj.actual_x;
            state(2) = obj.dot_x;
            state(3) = obj.actual_y;
            state(4) = obj.dot_y;
            state(5) = obj.theta;
            state(6) = obj.dot_theta;
            
        end
        
        function controls = get_controls(obj,t)
            controls = -[...
                obj.K*(obj.x-obj.vartheta_x),...
                obj.K*(obj.y-obj.vartheta_y),...
                obj.K_theta*(obj.theta-obj.vartheta_theta)+obj.alpha(t,obj.theta,obj.x,obj.y,obj.vartheta_x,obj.vartheta_y)...
                ];
            
        end
    end
    methods (Access = private)
        function summs= get_summs(obj,robots)
            k=length(robots);
            summs_x = 0;
            summs_y = 0;
            summs_theta = 0;
            for i = 1:k
                if class(robots(i)) ~= class(obj)
                    error("The robot is not a PlaneDoubleIntObj");
                end
                if obj.connections(robots(i).index)
                    summs_x =summs_x + obj.vartheta_x - robots(i).vartheta_x ;
                    summs_y =summs_y + obj.vartheta_y - robots(i).vartheta_y ;
                    summs_theta =summs_theta+ obj.vartheta_theta - robots(i).vartheta_theta ;
                end
            end
            summs=[summs_x;summs_y;summs_theta];
        end
        function set_vartheta(obj, vartheta)
            obj.vartheta_x=vartheta(1);
            obj.vartheta_y=vartheta(2);
            obj.vartheta_theta=vartheta(3);
        end
    end
end