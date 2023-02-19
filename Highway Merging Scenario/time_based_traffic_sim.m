classdef time_based_traffic_sim < matlab.System & matlab.system.mixin.Propagates
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        % Sensors resolution
       
        n;
        
        alive;
        velocity;
        pos;
        
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants 
            obj.n = 10;
            
            obj.alive = zeros(1,obj.n);
            obj.velocity = zeros(1,obj.n);
            obj.pos = zeros(3,obj.n);
            
            
        end

        function [ALIVE, POS, VEL] = stepImpl(obj)
                       
            pos = obj.num_vehicles;
            
            long_velocity = zeros(1,obj.num_vehicles);
            
            pause(1)
            
            
        end
        
        function [pos,long_velocity] = isOutputComplexImpl(~)
            
            pos =false;
            long_velocity = false;

        end
        
        function [pos,long_velocity] = getOutputSizeImpl(obj)
            pos = [1 1];
            long_velocity = [1,obj.num_vehicles]

        end
        
        function [pos,long_velocity] = getOutputDataTypeImpl(~)
            pos = 'double';
            long_velocity = 'double';
        end

        function [pos,long_velocity] = isOutputFixedSizeImpl(~)
            pos = true;
            long_velocity = false;
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
    
    methods(Access= public)
        function delete(obj)
            % Delete the car from the Carla world
    
        end
    end
end
