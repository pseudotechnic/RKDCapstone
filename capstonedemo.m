function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".

%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.

% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function [] = command_trajectory(robot, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();

  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    robot.set(cmd);

    % Wait a little bit to send at ~100Hz.

    
    
    
    
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robot.set(cmd);
end

% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end

% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end

% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');

gains.jenga_gains.positionKp = [1 4 5 2 0.5];
gains.jenga_gains.positionKi = [0 0 0 0 0];
gains.jenga_gains.positionKd = [0.01 0.01 0.01 0.0 0];
gains.jenga_gains.positionFF = [0 0.05 0.05 0 0];

robot.set('gains', gains.jenga_gains);

robot.set('gains', gains.jenga_gains);

robot.set('gains', gains.jenga_gains);


%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz
frequency = 100;


pickupapp = [0.1584, 0.8590, 1.5832, 1.5843, 0.4123]';
pickuppt = [0.1480, 0.7779, 1.5950, 1.4827, 0.4123]';
midpoint = [0.5757, 0.9807, 1.6641, 0.7339, 0.4123]';

b11app = [1.0046 0.7256 1.5461 0.9065 0.4154-pi/4]';
b11plc = [1.0046 0.7354 1.5463 0.9013 0.4154-pi/4]';

b12app = [0.9646 0.7856 1.6391 0.8665 0.4154-pi/4]';
b12plc = [0.9646 0.7414 1.6363 0.9213 0.4154-pi/4]';

b13app = [0.9404 0.7721 1.6675 0.9094 0.4154-pi/4]';
b13plc = [0.9404 0.7422 1.6602 0.9073 0.4154-pi/4]';


% off towards robot
b21app = [ 1.0211    0.78010   1.5891     0.7851    0.4154+pi/4]';
b21plc = [ 1.0198    0.7451    1.5825   0.9991   0.4154+pi/4]';

% too close to 21
b22app = [0.9671    0.7818    1.5502    0.9134 0.4154+pi/4]';
b22plc = [0.9674    0.7982    1.6080    0.9220 0.4154+pi/4]';

b23app = [0.9425    0.7561    1.4873    0.7749 0.4153+pi/4]';
b23plc = [0.9398    0.6925    1.4849    0.8136 0.4154+pi/4]';


b31app = [1.0413    0.7421    1.4555    0.7284 0.4154-pi/4]';
b31plc = [1.0399    0.7164    1.4603    0.7528 0.4154-pi/4]';

b32app = [0.9595    0.8598    1.6540    0.9313 0.4154-pi/4]';
b32plc = [0.9594    0.8016    1.6604    0.9699 0.4154-pi/4]';

b33app = [0.9256    0.8801    1.6910    0.9071 0.4154-pi/4]';
b33plc = [0.9227    0.8172    1.6984    0.9531 0.4154-pi/4]';


% b31app = [0.9804 0.9221 1.6075 0.9294 0.4154-pi/4]';
% b31plc = [0.9804 0.8822 1.5902 0.9273 0.4154-pi/4]';
% b32app = [0.9646 0.9456 1.6391 0.8665 0.4154-pi/4]';
% b32plc = [0.9646 0.9014 1.6363 0.9213 0.4154-pi/4]';
% b33app = [0.9272 0.9623 1.7015 0.9194 0.4153-pi/4]';
% b33plc = [0.9272 0.8818 1.7026 0.9744 0.4154-pi/4]';


dropapp = [b11app b12app b13app b21app b22app b23app b32app b33app b33app];
drop = [b11plc b12plc b13plc b21plc b22plc b23plc b32plc b33plc b33plc];

% for i=1:size(dropapp,2)
%     dropapp(4,i) = (dropapp(3,i) - (pi/2 - dropapp(2,i)));
%     drop(4,i) = (drop(3,i) - (pi/2 - drop(2,i)));
% end

trajectory = trajectory_spline([initial_thetas midpoint pickupapp], [0, 2, 3], frequency);
command_trajectory(robot, trajectory, frequency);

for l=1:size(dropapp,2)
%pickup
    
    trajectory = trajectory_spline([pickupapp pickuppt], [0, 1], frequency);
    command_trajectory(robot, trajectory, frequency);
    pick(gripper);
    pause(0.75);
    %drop
    trajectory = trajectory_spline([pickuppt pickupapp], [0, 1], frequency);
    command_trajectory(robot, trajectory, frequency);
    trajectory = trajectory_spline([pickupapp midpoint dropapp(:,l)], [0, 1, 2], frequency);
    command_trajectory(robot, trajectory, frequency);  
    trajectory = trajectory_spline([dropapp(:,l) drop(:,l)], [0, 1], frequency);
    command_trajectory(robot, trajectory, frequency);
    %% Place the object
    place(gripper);
    pause(0.75);
    
    trajectory = trajectory_spline([drop(:,l) dropapp(:,l)], [0, 1], frequency);
    command_trajectory(robot, trajectory, frequency);
    trajectory = trajectory_spline([dropapp(:,l) midpoint pickupapp], [0, 1, 2], frequency);
    command_trajectory(robot, trajectory, frequency);
end

trajectory = trajectory_spline([pickupapp midpoint], [0, 1], frequency);
    command_trajectory(robot, trajectory, frequency); 




% We define our "position 1", "position 2", and a "midpoint" waypoints
% here. We found these points by calling robot.getNextFeedback() from the MATLAB
% command line, and tweaking the results as necessary.
%position_1 = [-0.6 0.86 1.4 2.05 0]';
%position_2 = [0.32 0.70 1 2.41 0]';

% Base, Elbow, Shoulder, Wrist1, Wrist2

%position_1 = [0.1480, 0.7979, 1.5950, 1.4827, 0.4123]';
%position_2 = [0.9757, 0.7047, 1.5647, 0.9035, 0.4126]';

% We keep the last joint equal to the first to ensure the block does not rotate
% as we move. Note this joint points in the opposite direction as the base. For
% position 2, we want to rotate 1/4 turn, so we add pi/2.
%position_1(5) = position_1(1);
%position_2(5) = position_2(1) + pi/2;

% Add a midpoint for the trajectory, so the robot does not just drag the piece
% across the table.
%midpoint = [-0.1906 1.402 1.584 0.4989 0]';

%midpoint = [0.5757, 0.9807, 1.6641, 0.7339, 0.4123]';

%midpoint(5) = position_1(5)*0.5 + position_2(5)*0.5;

% Create a set of "approach" angles that let us have a slow "final approach" to
% the actual pick and place location.  This can increase accuracy and reduce
% issues where straight-line-configuration-space trajectories make the end
% effector hit the table
%position_1_approach = [-0.5192 0.8585 1 1.179 position_1(5)]';
%position_2_approach = [0.3 0.7152 1 2.3 position_2(5)]';

%position_1_approach = [0.1584, 0.8590, 1.5832, 1.5843, 0.4123]';
%position_2_approach = [0.9093, 0.7556, 1.6117, 0.9056, 0.4124]';


%% Stop logging, and plot results
robot.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end
