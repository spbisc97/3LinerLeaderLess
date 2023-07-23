classdef PlaneDoubleIntObj < handle
    properties (Access = public)
        x
        y
        
        vartheta_x
        vartheta_y
        
        vartheta_dot_x
        vartheta_dot_y
        
        vartheta_ddot_x
        vartheta_ddot_y
        
        index
        connections=zeros(1,128);
    end
    
    properties (Access = private)
        dot_x
        dot_y
        P=0.0005;
        D=0.03;
        K=0.001;
    end
    
    methods (Access = public)
        function obj = PlaneDoubleIntObj(index)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
            obj.vartheta_x=0;
            obj.vartheta_y=0;
            obj.vartheta_dot_x=0;
            obj.vartheta_dot_y=0;
            obj.vartheta_ddot_x=0;
            obj.vartheta_ddot_y=0;
            obj.index = index;
        end
        
        function obj = reset(obj)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
            obj.vartheta_x=0;
            obj.vartheta_y=0;
            obj.vartheta_dot_x=0;
            obj.vartheta_dot_y=0;
            obj.vartheta_ddot_x=0;
            obj.vartheta_ddot_y=0;
        end
        
        function obj = update(obj, u, dt, robots)

            summs=get_summs(obj,robots);
            u_x=u(1);
            u_y=u(2);
            obj.x = obj.x + obj.dot_x * dt;
            obj.dot_x = obj.dot_x + u_x * dt;
            obj.y = obj.y + obj.dot_y * dt;
            obj.dot_y = obj.dot_y + u_y * dt;

            
            obj.vartheta_x=obj.vartheta_x+obj.vartheta_dot_x*dt;
            obj.vartheta_y=obj.vartheta_y+obj.vartheta_dot_y*dt;
            obj.vartheta_dot_x=obj.vartheta_dot_x+obj.vartheta_ddot_x*dt;
            obj.vartheta_dot_y=obj.vartheta_dot_y+obj.vartheta_ddot_y*dt;



            obj.vartheta_ddot_x=-obj.P*summs(1)-obj.D*obj.vartheta_dot_x-u_x;
            obj.vartheta_ddot_y=-obj.P*summs(2)-obj.D*obj.vartheta_dot_y-u_y;

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
            obj.x = state(1);
            obj.dot_x = state(2);
            obj.y = state(3);
            obj.dot_y = state(4);
        end
        
        function obj = set_xy(obj, p)
            obj.x = p(1);
            obj.y = p(2);
        end
        
        function obj = set_dot_x(obj, v)
            obj.dot_x = v(1);
            obj.dot_y = v(2);
        end
        
        function state = get_state(obj)
            state(1) = obj.x;
            state(2) = obj.dot_x;
            state(3) = obj.y;
            state(4) = obj.dot_y;
        end
        
        function controls = get_controls(obj)
            controls = -obj.K*[obj.x-obj.vartheta_x,obj.y-obj.vartheta_y];
        end
    end
    methods (Access = private)
        function summs= get_summs(obj,robots)
            k=length(robots);
            summs_x = 0;
            summs_y = 0;
            for i = 1:k
                if class(robots(i)) ~= class(obj)
                    error("The robot is not a PlaneDoubleIntObj");
                end
                if obj.connections(robots(i).index)
                    summs_x =summs_x + obj.vartheta_x - robots(i).vartheta_x ;
                    summs_y =summs_y + obj.vartheta_y - robots(i).vartheta_y ;
                end
            end
            summs=[summs_x;summs_y];
        end

        end
end