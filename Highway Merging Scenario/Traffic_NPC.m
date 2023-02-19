classdef Traffic_NPC < matlab.System & matlab.system.mixin.Propagates
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
        world;
        spawn;

        
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            port = int16(2000);
            client = py.carla.Client('localhost', port);
            client.set_timeout(10.0);
            obj.world = client.get_world();
            
            
            obj.spawn = true;
            
            
            
            
         
            
        end

        function [pos,long_velocity] = stepImpl(obj,color,init_pos,throttle,brake,steer,restart_pos)
            
            if obj.spawn == true
                % Spawn Vehicle
                transform = py.carla.Transform(py.carla.Location(init_pos(1), init_pos(2), init_pos(3)),py.carla.Rotation(0, 180, 0));
                vehicle_bp = obj.world.get_blueprint_library().find("vehicle.toyota.prius");
                car_color = "(" + num2str(color(1)) + "," + num2str(color(2)) + "," + num2str(color(3)) + ")" ;
                vehicle_bp.set_attribute('color', car_color);
                obj.tesla = obj.world.spawn_actor(vehicle_bp,transform );
                
               
                %Control
                obj.control = py.carla.VehicleControl();
                
                obj.spawn = false;
            end

            
            x_position = obj.tesla.get_location().x;
            x_velocity = obj.tesla.get_velocity().x;
            y_position = obj.tesla.get_location().y;
            y_velocity = obj.tesla.get_velocity().y;
            
            
            yaw = obj.tesla.get_transform().rotation.yaw;
            
            long_velocity =abs( x_velocity * cosd(yaw) + y_velocity * sind(yaw));
            
            pos = [x_position y_position yaw*pi/180];
            
            obj.control.throttle = throttle;
            obj.control.brake = brake;
            obj.control.steer = steer;
           
            %keyboard_control(obj.control)
            obj.tesla.apply_control(obj.control); 
            
            if  x_position < -0
                transform = py.carla.Transform(py.carla.Location(restart_pos(1), restart_pos(2), restart_pos(3)),py.carla.Rotation(0, 180, 0));
                obj.tesla.set_transform(transform);
            end
            
        end
        
        function [pos,long_velocity] = isOutputComplexImpl(~)
            
            pos =false;
            long_velocity = false;

        end
        
        function [pos,long_velocity] = getOutputSizeImpl(~)
            pos = [1 3];
            long_velocity = [1 1];

        end
        
        function [pos,long_velocity] = getOutputDataTypeImpl(~)
            pos = 'double';
            long_velocity = 'double';
        end

        function [pos,long_velocity] = isOutputFixedSizeImpl(~)
            pos = true;
            long_velocity = true;
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
