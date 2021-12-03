classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_masses
        joint_masses
        dh_parameters
    end
    
    methods
        %% Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(dh_parameters, link_masses, joint_masses)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(dh_parameters, 2) ~= 4
                error('Invalid dh_parameters: Should be a dof x 4 matrix, is %dx%d.', size(dh_parameters, 1), size(dh_parameters, 2));
            end
            
            if size(link_masses, 2) ~= 1
                error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
                error('Invalid joint_masses: Should be a column vector.');
            end
            
            robot.dof = size(dh_parameters, 1);
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            robot.dh_parameters = dh_parameters;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
        end
        
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        
        %% Foward Kinematics        
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).
            
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 4x4 matrix frames(:,:,end).
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            for i = 1:n
                % get one row of params
                dh = robot.dh_parameters(i,:);
                % update thetas as variables
                dh(4) = rad2deg(thetas(i));
                
                % extract dh params
                if i == 1
                    frames(:,:,i) = [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
                else
                    frames(:,:,i) = frames(:,:,i-1) * [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
                end
            end
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
        
        % Returns [x; y; z; psi; theta; phi] for the end effector given a
        % set of joint angles. Remember that psi is the roll, theta is the
        % pitch, and phi is the yaw angle.
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
            
            % Extract the components of the end_effector position and
            % orientation.
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % translation component of transform
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);
            
            % rotation based on z being forward
            % x is phi, z is psi
            phi = (atan2(H_0_ee(3,2), H_0_ee(3,3)));
            theta = (atan2(-H_0_ee(3,1), sqrt(H_0_ee(3,2)^2 + H_0_ee(3,3)^2)));
            psi = (atan2(H_0_ee(2,1),H_0_ee(1,1)));
            % --------------- END STUDENT SECTION ------------------------------------
            
            % Pack them up nicely.
            ee = [x; y; z; psi; theta; phi];
        end
        
        % Shorthand for returning the end effector position and orientation.
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        %% Jacobians
        
        function jacobians = jacobians_numerical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            epsilon = 0.001;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            for j=1:robot.dof
               thetap = thetas;
               thetap(j) = thetap(j) + epsilon;
               thetam = thetas;
               thetam(j) = thetam(j) + epsilon;
               
               framep = robot.fk(thetap);
               framepx = framep(1,6,:);
               framepy = framep(2,6,:);
               framepz = framep(3,6,:);
               frameppsi = framep(4,6,:);
               frameptheta = framep(5,6,:);
               framepphi = framep(6,6,:);
               
               framem = robot.fk(thetam);
               framemx = framem(1,6,:);
               framemy = framem(2,6,:);
               framemz = framem(3,6,:);
               framempsi = framem(4,6,:);
               framemtheta = framem(5,6,:);
               framemphi = framem(6,6,:);
               
               for i=1:size(framep,6)
                jacobians(1, j, i) = (framepx(frame) - framemx(frame)) / (2 * epsilon);
                jacobians(2, j, i) = (framepy(frame) - framemy(frame)) / (2 * epsilon);
                jacobians(3, j, i) = (framepz(frame) - framemz(frame)) / (2 * epsilon);
                jacobians(4, j, i) = (frameppsi(frame) - framempsi(frame)) / (2 * epsilon);
                jacobians(5, j, i) = (frameptheta(frame) - framemtheta(frame)) / (2 * epsilon);
                jacobians(6, j, i) = (framepphi(frame) - framemphi(frame)) / (2 * epsilon);
               
               end
            end
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function jacobians = jacobians_analytical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % TODO build up the jacobian using the analytical
            % convention from lecture
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        
        %% Inverse Kinematics
        
        function thetas = inverse_kinematics_graddescent(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;
            
            % Step size for gradient update
            step_size = 0.0000005;
            
            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;
            
            % Also, limit to a maximum number of iterations.
            max_iter = 50000;
            num_iter = 0;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)
                
                % Compute the gradient for either an [x;y;z] goal or an
                % [x; y; z; psi; theta; phi] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
                if(size(goal_position, 1) == 3)
                    ee = robot.end_effector(thetas);
                    ee21 = zeros(3,1);
                    ee21(1) = ee(1);
                    ee21(2) = ee(2);
                    ee21(3) = ee(3);
                    jac = robot.jacobians(thetas);
                    jac22 = jac(1:3,:,end);
                    cost_gradient = transpose(jac22)*(ee21-goal_position);
                else
                    jac = robot.jacobians(thetas);
                    cost_gradient = transpose(jac(:,:,end))*(ee-goal_position);
                end
                
                
                % Update 'thetas'
                % TODO
                thetas = thetas - step_size * cost_gradient;
                
                % Check stopping condition, and return if it is met.
                % TODO
                
                if(norm(cost_gradient) == stopping_condition)
                    return;
                end
                
                num_iter = num_iter + 1;
            end
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function cost = cost_function(robot, thetas, goal_position)
            % Cost function for fmincon
            current_pose = robot.ee(thetas);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            % --------------- END STUDENT SECTION ------------------------------------
            
        end
        
        function thetas = inverse_kinematics_numopt(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function. Using built in optimization (fmincon)
            
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            
            fun = @(thetas)robot.cost_function(thetas, goal_position);
            
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = inverse_kinematics_analytical(robot, goal_position)
            % Returns the joint angles using an analytical approach to
            % inverse kinematics
            % Note: Kinematics Decoupling might be very useful for this
            % question
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
            %    error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            %end
            
            thetas = zeros(robot.dof, 1);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = trajectory_to_thetas(robot, trajectory)
            thetas(:, 1) = robot.inverse_kinematics_analytical(trajectory(:, 1));
            for theta_col=2:size(trajectory, 2)
                thetas(:, theta_col) = robot.inverse_kinematics_analytical(trajectory(:, theta_col));
            end
        end
        
    end
end