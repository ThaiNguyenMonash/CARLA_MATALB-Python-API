classdef merger < matlab.System & matlab.system.mixin.Propagates
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
        width = 960;
        height = 540;
        
        tesla;
        
        front_cam_seg;
        module_front_cam_seg;
        left_cam_seg;
        module_left_cam_seg;
        back_cam_seg;
        module_back_cam_seg;
        
        front_cam_depth;
        module_front_cam_depth;
        left_cam_depth;
        module_left_cam_depth; 
        back_cam_depth;
        module_back_cam_depth;
        
        top_cam_rgb;
        module_top_cam_rgb;
 
        
        
        control;
        
        focal_length;
        center_x;
        center_y;
        cam_matrix;
        
        
        
        world;
        
        id_count;
        prev_time;
        targets;
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            port = int16(2000);
            client = py.carla.Client('localhost', port);
            client.set_timeout(10.0);
            obj.world = client.get_world();
            
            % Spawn Vehicle
            transform = py.carla.Transform(py.carla.Location(130, -35, 9.6),py.carla.Rotation(0, 105, 0));
            vehicle_bp = obj.world.get_blueprint_library().find("vehicle.tesla.model3");
            obj.tesla = obj.world.spawn_actor(vehicle_bp,transform );
            
%             % Collision Detector
%             blueprint = obj.world.get_blueprint_library().find('sensor.other.collision');
%             obj.collision_sensor = obj.world.spawn_actor(blueprint, py.carla.Transform, pyargs('attach_to',obj.tesla));
%             collision_sensor.listen(lambda x , xdata: function_handler(event))

            % Seg Blueprint
            blueprint = obj.world.get_blueprint_library().find('sensor.camera.semantic_segmentation');
            blueprint.set_attribute('image_size_x', num2str(obj.width));
            blueprint.set_attribute('image_size_y', num2str(obj.height));
            blueprint.set_attribute('fov', '90')
            obj.focal_length = obj.width /(2 * tan(90 * pi / 360));
            obj.center_x = obj.width / 2;
            obj.center_y = obj.height / 2;
            obj.cam_matrix = [obj.focal_length 0 obj.center_x; 0 obj.focal_length obj.center_y; 0 0 1];
            type = "semantic_segmentation_mask";
            
            
            
            
            % Front Cam Seg
            transform = py.carla.Transform(py.carla.Location(pyargs('x',0.8, 'z',1.7)));
            obj.front_cam_seg = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_front_cam_seg = sensorBind(obj.front_cam_seg, "CAM_front_cam_seg", type, "array");
            
            %Left Camera Seg
            transform = py.carla.Transform(py.carla.Location(0,-0.2,1.7),py.carla.Rotation(0, 270, 0));            
            obj.left_cam_seg = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_left_cam_seg = sensorBind(obj.left_cam_seg, "CAM_left_cam_seg", type, "array");
            
            %Back Camera Seg
            transform = py.carla.Transform(py.carla.Location(-1.5,0,1.7),py.carla.Rotation(0, 180, 0));            
            obj.back_cam_seg = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_back_cam_seg = sensorBind(obj.back_cam_seg, "CAM_back_cam_seg", type, "array");
            
            % Depth Blueprint
            blueprint = obj.world.get_blueprint_library().find('sensor.camera.depth');
            blueprint.set_attribute('image_size_x', num2str(obj.width));
            blueprint.set_attribute('image_size_y', num2str(obj.height));
            type = "depth";
            
            % Front Cam Depth
            transform = py.carla.Transform(py.carla.Location(pyargs('x',0.8, 'z',1.7)));
            obj.front_cam_depth = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_front_cam_depth = sensorBind(obj.front_cam_depth, "CAM_front_cam_depth", type, "array");
            
            %Left Camera Depth
            transform = py.carla.Transform(py.carla.Location(0,-0.2,1.7),py.carla.Rotation(0, 270, 0));            
            obj.left_cam_depth = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_left_cam_depth = sensorBind(obj.left_cam_depth, "CAM_left_cam_depth", type, "array");
            
            %Back Camera Depth
            transform = py.carla.Transform(py.carla.Location(-1.5,0,1.7),py.carla.Rotation(0, 180, 0));          
            obj.back_cam_depth = obj.world.spawn_actor(blueprint, transform, pyargs('attach_to',obj.tesla));
            obj.module_back_cam_depth = sensorBind(obj.back_cam_depth, "CAM_back_cam_depth", type, "array");
            
            % RGB Blueprint
            blueprint = obj.world.get_blueprint_library().find('sensor.camera.rgb');
            blueprint.set_attribute('image_size_x', num2str(obj.width));
            blueprint.set_attribute('image_size_y', num2str(obj.height/2));
            type = "rgb";
            
            transform = py.carla.Transform(py.carla.Location(50,-5,100),py.carla.Rotation(-90,0,90));          
            obj.top_cam_rgb = obj.world.spawn_actor(blueprint, transform);
            obj.module_top_cam_rgb = sensorBind(obj.top_cam_rgb, "CAM_top_cam_rgb", type, "array");
            
            
            
            %Control
            obj.control = py.carla.VehicleControl();
            
            %Targets
            fclose(fopen('targets.txt', 'w'));
            obj.id_count = 1;
            obj.targets = [];
            obj.prev_time = 0;
            
         
            
        end

        function [FRONT_CAM_SEG,LEFT_CAM_SEG,BACK_CAM_SEG,TOP_CAM_RGB,pos,long_velocity,ang_velocity] = stepImpl(obj,throttle,brake,steer)
            
            obj.world.tick()
            
            % get outputs
            x_position = obj.tesla.get_location().x;
            x_velocity = obj.tesla.get_velocity().x;
            y_position = obj.tesla.get_location().y;
            y_velocity = obj.tesla.get_velocity().y;
            
            
            yaw = obj.tesla.get_transform().rotation.yaw;
            
            long_velocity =abs( x_velocity * cosd(yaw) + y_velocity * sind(yaw));
            
            ang_velocity = obj.tesla.get_angular_velocity().z;
            
            pos = [x_position y_position yaw*pi/180];
            
            % Cameras
            obj.targets = [];
            
            TOP_CAM_RGB = uint8(py.getattr(obj.module_top_cam_rgb, "array"));
            
            
            
            FRONT_CAM_SEG = uint8(py.getattr(obj.module_front_cam_seg, "array"));
            FRONT_CAM_DEPTH = double(py.getattr(obj.module_front_cam_depth, 'array'));
            [L,N] = bwlabel(eq(FRONT_CAM_SEG(:,:,1),0) .* eq(FRONT_CAM_SEG(:,:,2),0) .* eq(FRONT_CAM_SEG(:,:,3),142));
            FRONT_CAM_SEG = uint8(eq(FRONT_CAM_SEG(:,:,1),0) .* eq(FRONT_CAM_SEG(:,:,2),0) .* eq(FRONT_CAM_SEG(:,:,3),142));
            for n = 1:N
        
               s = regionprops(eq(L,n),'BoundingBox') ;
               bb = cat(1,s.BoundingBox);
               
               if bb(3) < 5 || bb(4) < 5
                   continue
               end
%                
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
               
%                disp("Bounding Box:")
%                disp(bb)
               center = [round(bb(1)+0.5*bb(3)),round(bb(2)+0.5*bb(4))];
%                disp("Center:")
%                disp(center)
               
               depth = FRONT_CAM_DEPTH(center(2),center(1));
%                disp("Depth:")
%                disp(depth)
               
               pos_local = inv(obj.cam_matrix)*[center(1);center(2);1] * (depth) ;
               pos_local(1) = pos_local(1)*-1;
               pos_local(2) = pos_local(3);
               pos_local(3) = 1;
%                disp("pos_relative to camera")
%                disp(pos_local)
               
%                disp("pos_relative_to car")
               theta = -1*pi/2;
               cam_pos = [cos(theta) ,-sin(theta) ,0.8 ;sin(theta) ,cos(theta), 0; 0, 0, 1]*pos_local;
%                disp(cam_pos)
               
%                disp("pos_relative_to_world")
               theta = yaw*pi/180;
               target_pos = [cos(theta) ,-sin(theta) ,x_position ;sin(theta) ,cos(theta), y_position; 0, 0, 1]*cam_pos;
%                disp(target_pos)

               obj.targets = [obj.targets,target_pos(1:2)];
              
            end
            
            LEFT_CAM_SEG = uint8(py.getattr(obj.module_left_cam_seg, 'array'));
            LEFT_CAM_DEPTH = double(py.getattr(obj.module_left_cam_depth, 'array'));
            [L,N] = bwlabel(eq(LEFT_CAM_SEG(:,:,1),0) .* eq(LEFT_CAM_SEG(:,:,2),0) .* eq(LEFT_CAM_SEG(:,:,3),142));

            
            for n = 1:N
        
               s = regionprops(eq(L,n),'BoundingBox') ;
               bb = cat(1,s.BoundingBox);
               
               if bb(3) < 5 || bb(4) < 5
                   continue
               end
               
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                
%                disp("Bounding Box:")
%                disp(bb)
               center = [round(bb(1)+0.5*bb(3)),round(bb(2)+0.5*bb(4))];
%                disp("Center:")
%                disp(center)
               
               depth = LEFT_CAM_DEPTH(center(2),center(1));
%                disp("Depth:")
%                disp(depth)
               
               pos_local = inv(obj.cam_matrix)*[center(1);center(2);1] * (depth) ;
               pos_local(1) = pos_local(1)*-1;
               pos_local(2) = pos_local(3);
               pos_local(3) = 1;
%                disp("pos_relative to camera")
%                disp(pos_local)
%                
%                disp("pos_relative_to car")
               theta = -1*pi;
               cam_pos = [cos(theta) ,-sin(theta) ,0 ;sin(theta) ,cos(theta), -0.2; 0, 0, 1]*pos_local;
%                disp(cam_pos)
               
%                disp("pos_relative_to_world")
               theta = yaw*pi/180;
               target_pos = [cos(theta) ,-sin(theta) ,x_position ;sin(theta) ,cos(theta), y_position; 0, 0, 1]*cam_pos;
%                disp(target_pos)

               obj.targets = [obj.targets,target_pos(1:2)];
              
            end
            
            
            BACK_CAM_SEG = uint8(py.getattr(obj.module_back_cam_seg, 'array'));
            BACK_CAM_DEPTH = double(py.getattr(obj.module_back_cam_depth, 'array'));
            [L,N] = bwlabel(eq(BACK_CAM_SEG(:,:,1),0) .* eq(BACK_CAM_SEG(:,:,2),0) .* eq(BACK_CAM_SEG(:,:,3),142));
            
            for n = 1:N
        
               s = regionprops(eq(L,n),'BoundingBox') ;
               bb = cat(1,s.BoundingBox);
               
               if bb(3) < 5 || bb(4) < 5
                   continue
               end
%                
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
%                disp("------------------")
               
%                disp("Bounding Box:")
%                disp(bb)
               center = [round(bb(1)+0.5*bb(3)),round(bb(2)+0.5*bb(4))];
%                disp("Center:")
%                disp(center)
               
               depth = BACK_CAM_DEPTH(center(2),center(1));
%                disp("Depth:")
%                disp(depth)
               
               pos_local = inv(obj.cam_matrix)*[center(1);center(2);1] * (depth) ;
               pos_local(1) = pos_local(1)*-1;
               pos_local(2) = pos_local(3);
               pos_local(3) = 1;
%                disp("pos_relative to camera")
%                disp(pos_local)
               
%                disp("pos_relative_to car")
               theta = pi/2;
               cam_pos = [cos(theta) ,-sin(theta) ,-1.5 ;sin(theta) ,cos(theta), 0; 0, 0, 1]*pos_local;
%                disp(cam_pos)
               
%                disp("pos_relative_to_world")
               theta = yaw*pi/180;
               target_pos = [cos(theta) ,-sin(theta) ,x_position ;sin(theta) ,cos(theta), y_position; 0, 0, 1]*cam_pos;
%                disp(target_pos)

               obj.targets = [obj.targets,target_pos(1:2)];
              
            end
            
            disp("START")
            if ~isempty(obj.targets)
                
                default_speed = 20;
                output = [];
                
                
                for a = 1:length(obj.targets(1,:))
                   
                    x = obj.targets(1,a);
                    y = obj.targets(2,a);
                                                          
                    s = dir('targets.txt');
                    
                    detected_before = false;
                    
                    
                    if s.bytes ~= 0                    
                        prev_targets = csvread('targets.txt');
                        distance = sqrt( (prev_targets(:,2) - x ).^2 + (prev_targets(:,3) - y).^2 );
                        
                        disp("Distance")
                        disp(distance)
                        
                        [min_dist,index] = min(distance);
                        
                        disp("Min Distance")
                        disp(min_dist)
                        
                        
                        if min_dist < 20
                            detected_before = true;
                            vel = min_dist/(getCurrentTime(obj)- obj.prev_time);
                            disp(getCurrentTime(obj)- obj.prev_time)
                            output = [output; prev_targets(index,1) x y vel];
                            disp("Same")
                            disp(a)
                        end
                        
                    end
                    
                    if detected_before == false
                        
                        if s.bytes ~= 0
                            prev_targets = csvread('targets.txt');
                            A = prev_targets(:,1);
                            if min(A) > 1
                                next_id = 1;
                            else
                                ind = find(histcounts(A)== 0);
                                if isempty(ind)
                                    next_id = max(A) + 1;
                                else
                                    next_id = ind(1);
                                end
                            end
                            obj.id_count = next_id;                            
                        else
                            obj.id_count =  1;
                        end

                        output = [output; obj.id_count x y default_speed];
                        
                        disp("Not Same")
                        disp(a)
                    end                    
                end
                
                for id = min(output(:,1)): max(output(:,1))
                    result = find(output(:,1)==id);
                    if length(result) > 1
                        for index = 2:length(result)

                            distance = sqrt( (output(result,2) - output(result(index),2) ).^2 + ( output(result,3) - output(result(index),3) ).^2 );
                            max_dist = max(distance);

                            if max_dist > 5
                                
                                A = output(:,1);
                                if min(A) > 1
                                    next_id = 1;
                                else
                                    ind = find(histcounts(A)== 0);
                                    if isempty(ind)
                                        next_id = max(A) + 1;
                                    else
                                        next_id = ind(1);
                                    end
                                end

                                obj.id_count = next_id;
                                
                                output(result(index),1) = obj.id_count;
                                obj.id_count = obj.id_count + 1;
                            end

                        end
                    end
                end


                disp(output)
                csvwrite('targets.txt',output);
            else
                fclose(fopen('targets.txt', 'w'));
            end
            
            disp("END")
            
            
            
            
            
            
            
            
            
            obj.control.throttle = throttle;
            obj.control.brake = brake;
            obj.control.steer = steer;
            
            if x_position < 0
                transform = py.carla.Transform(py.carla.Location(130, -35, 9.6),py.carla.Rotation(0, 105, 0));
                obj.tesla.set_transform(transform);
                obj.control.throttle = 0;
                obj.control.brake = 1;
                obj.control.steer = 0;
                pause(1) 
            end
            
            obj.tesla.apply_control(obj.control); 
            obj.prev_time = getCurrentTime(obj);
            
        end
        
        function [FRONT_CAM_SEG,LEFT_CAM_SEG,BACK_CAM_SEG,TOP_CAM_RGB,pos,long_velocity,ang_velocity] = isOutputComplexImpl(~)
            
            FRONT_CAM_SEG = false;
            LEFT_CAM_SEG = false;
            BACK_CAM_SEG = false;
            TOP_CAM_RGB = false;
            
            pos =false;
            long_velocity = false;
            ang_velocity = false;

        end
        
        function [FRONT_CAM_SEG,LEFT_CAM_SEG,BACK_CAM_SEG,TOP_CAM_RGB,pos,long_velocity,ang_velocity] = getOutputSizeImpl(obj)
            
            FRONT_CAM_SEG = [obj.height obj.width 1];
            LEFT_CAM_SEG= [obj.height obj.width 3];
            BACK_CAM_SEG = [obj.height obj.width 3];
            TOP_CAM_RGB = [obj.height/2,obj.width,3];
            
            pos = [1 3];
            long_velocity = [1 1];
            ang_velocity = [1 1];

        end
        
        function [FRONT_CAM_SEG,LEFT_CAM_SEG,BACK_CAM_SEG,TOP_CAM_RGB,pos,long_velocity,ang_velocity] = getOutputDataTypeImpl(~)
            
            FRONT_CAM_SEG = 'uint8';
            LEFT_CAM_SEG = 'uint8';
            BACK_CAM_SEG = 'uint8';
            TOP_CAM_RGB = 'uint8';
            
            pos = 'double';
            long_velocity = 'double';
            ang_velocity = 'double';
            
        end

        function [FRONT_CAM_SEG,LEFT_CAM_SEG,BACK_CAM_SEG,TOP_CAM_RGB,pos,long_velocity,ang_velocity] = isOutputFixedSizeImpl(~)
            
            FRONT_CAM_SEG = true;
            LEFT_CAM_SEG = true;
            BACK_CAM_SEG = true;
            TOP_CAM_RGB = true;
            
            
            pos = true;
            long_velocity = true;
            ang_velocity = true;
            

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
            
            if ~isempty(obj.front_cam_seg)
                obj.front_cam_seg.destroy();
            end
            
            if ~isempty(obj.left_cam_seg)
                obj.left_cam_seg.destroy();
            end
            
            if ~isempty(obj.back_cam_seg)
                obj.back_cam_seg.destroy();
            end
            
            if ~isempty(obj.front_cam_depth)
                obj.front_cam_depth.destroy();
            end
            
            if ~isempty(obj.left_cam_depth)
                obj.left_cam_depth.destroy();
            end
            
            if ~isempty(obj.back_cam_depth)
                obj.back_cam_depth.destroy();
            end
            
            if ~isempty(obj.top_cam_rgb)
                obj.top_cam_rgb.destroy();
            end
            
        end
    end
end
