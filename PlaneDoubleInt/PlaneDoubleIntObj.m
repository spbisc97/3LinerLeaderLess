classdef PlaneDoubleIntObj < handle
    properties
        x
        dot_x
        y
        dot_y
        index
        P=0.005;
        D=0.3;
        connections=zeros(1,128);
    end

    methods
        function obj = PlaneDoubleIntObj(index)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
            obj.index = index;
        end
            
        function obj = update(obj, u, dt)
            u_x=u(1);
            u_y=u(2);
            obj.x = obj.x + obj.dot_x * dt;
            obj.dot_x = obj.dot_x + u_x * dt;
            obj.y = obj.y + obj.dot_y * dt;
            obj.dot_y = obj.dot_y + u_y * dt;
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

        function obj = reset(obj)
            obj.x = 0;
            obj.dot_x = 0;
            obj.y = 0;
            obj.dot_y = 0;
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

        function [controls] = get_control(obj,robots)
            k=length(robots);
            for i = 1:k
                if class(robots(i)) ~= class(obj)
                    error("The robot is not a PlaneDoubleIntObj");
                end            
            end
            summs_x = 0;
            summs_y = 0;
            for i = 1:k
                if obj.connections(robots(i).index)
                    summs_x =summs_x + obj.x - robots(i).x ;
                    summs_y =summs_y + obj.y - robots(i).y ;
                end
            end
            controls = [-obj.D*obj.dot_x - summs_x*obj.P,...
                -obj.D*obj.dot_y - summs_y*obj.P];
        end
    end
end