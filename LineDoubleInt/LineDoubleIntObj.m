classdef LineDoubleIntObj < handle
    properties
        position
        velocity
        index
        P=0.0003;
        D=0.02;
        connections=zeros(1,128);
    end

    methods
        function obj = LineDoubleIntObj(index)
            obj.position = 0;
            obj.velocity = 0;
            obj.index = index;
        end
            
        function obj = update(obj, u, dt)
            obj.position = obj.position + obj.velocity * dt;
            obj.velocity = obj.velocity + u * dt;
        end

        function obj=set_connections(obj,connections)
            for i=connections
                if mod(i,1) == 0
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
            obj.position = 0;
            obj.velocity = 0;
        end

        function obj = set_state(obj, state)
            obj.position = state(1);
            obj.velocity = state(2);
        end

        function obj = set_position(obj, position)
            obj.position = position;
        end

        function obj = set_velocity(obj, velocity)
            obj.velocity = velocity;
        end

        function state = get_state(obj)
            state(1) = obj.position;
            state(2) = obj.velocity;
        end

        function control = get_control(obj,robots)
            k=length(robots);
            for i = 1:k
                if class(robots(i)) ~= class(obj)
                    error("The robot is not a LineDoubleIntObj");
                end            
            end
            summs = 0;
            for i = 1:k
                if obj.connections(robots(i).index)
                    summs =summs + obj.position - robots(i).position ;
                end
            end
            control = -obj.D*obj.velocity - summs*obj.P;
        end
    end
end