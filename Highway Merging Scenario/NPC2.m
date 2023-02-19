classdef NPC2 < matlab.System & matlab.system.mixin.Propagates
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
        tesla;     
        control;
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            port = int16(2000);
            client = py.carla.Client('localhost', port);
            client.set_timeout(10.0);
            world = client.get_world();
            
            
            
            % Spawn Vehicle
            transform = py.carla.Transform(py.carla.Location(108, 6.5, 15),py.carla.Rotation(0, 180, 0));
            vehicle_bp = world.get_blueprint_library().find("vehicle.toyota.prius");
            obj.tesla = world.spawn_actor(vehicle_bp,transform );
            
            %Control
            obj.control = py.carla.VehicleControl();
            
            
            
         
            
        end

        function [pos,long_velocity,ang_velocity] = stepImpl(obj,throttle,brake,steer)
            
            x_position = obj.tesla.get_location().x;
            x_velocity = obj.tesla.get_velocity().x;
            y_position = obj.tesla.get_location().y;
            y_velocity = obj.tesla.get_velocity().y;
            
            
            yaw = obj.tesla.get_transform().rotation.yaw;
            
            long_velocity =abs( x_velocity * cosd(yaw) + y_velocity * sind(yaw));
            
            ang_velocity = obj.tesla.get_angular_velocity().z;
            
            pos = [x_position y_position yaw*pi/180];
            
            obj.control.throttle = throttle;
            obj.control.brake = brake;
            obj.control.steer = steer;
           
            %keyboard_control(obj.control)
            obj.tesla.apply_control(obj.control); 
            
            if  x_position < 0
                transform = py.carla.Transform(py.carla.Location(120, 6.5, 10.5),py.carla.Rotation(0, 180, 0));
                obj.tesla.set_transform(transform);
            end
            
        end
        
        function [pos,long_velocity,ang_velocity] = isOutputComplexImpl(~)
            
            pos =false;
            long_velocity = false;
            ang_velocity = false;

        end
        
        function [pos,long_velocity,ang_velocity] = getOutputSizeImpl(obj)
            pos = [1 3];
            long_velocity = [1 1];
            ang_velocity = [1 1];
        end
        
        function [pos,long_velocity,ang_velocity] = getOutputDataTypeImpl(~)
            pos = 'double';
            long_velocity = 'double';
            ang_velocity = 'double';
        end

        function [pos,long_velocity,ang_velocity] = isOutputFixedSizeImpl(~)
            pos = true;
            long_velocity = true;
            ang_velocity = true
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
    
    methods(Access= public)
        function delete(obj)
            % Delete the car from the Carla world
            if ~isempty(obj.tesla)
                obj.tesla.destroy();
            end               
        end
    end
end
