
% This code runs well on ROBOT 1


clc;
clear all;
mylego = legoev3('usb');
global PBase
global PERIOD
PERIOD = 0.1;

global grBase
global grLink
grBase = 3;   % gear ratio of base motor
grLink = 5;   % gear ration of link motor

% Link length in (mm)
global l1
l1=50;
global l2
l2=95;
global l3
l3=185;
global l4
l4=110;
global h;
h = 70;

global gripper
global link_motor
global base_motor
% assigned the ports to respective motors for actuation
gripper = motor(mylego, 'A');
link_motor = motor(mylego, 'B');
base_motor = motor(mylego, 'C');

global gripper_Status;
gripper_Status = 0;

% connection for reading status of touch sensors 1 and 3
touchSensor1 = touchSensor(mylego,1);
touchSensor3 = touchSensor(mylego,3);
sonic2 = sonicSensor(mylego,2);      % connect the sonic sensor to prot 2

% initially stop the motors 
stop(gripper);
stop(link_motor);
stop(base_motor);


global angC; 
angC = 0;
clc;

% set the speeds of motor
motorA_Speed = 55;
motorB_UpSpeed = 40;
motorB_DownSpeed = 15;
motorC_Speed = 30;
rc = 0;
touch_value1 = 0;    % set the value of touch sensor 1 to 0
touch_value3 = 0;    %  set the value of touch sensor 3 to 0
global angB;
global dist;
global angA;
start(gripper);   % Strat the gripper motor
gripper.Speed = 8;  % Open gripper
pause(2);
gripper.Speed = -8;  % close gripper
pause(2);

% set then gripper for operation such as open and close
gripper_operations("SET_INITIALLY", gripper)
pause(2);
gripper_operations("OPEN", gripper)
pause(2);
gripper_operations("CLOSE", gripper)



% at first set all heights for position A, B, and C to 0
dist_b_down = 0.0;
dist_c_down = 0.0;
dist_a_down = 0.0;


% Task 4: Command the manipulator to Home position
home(motorC_Speed, base_motor, link_motor, touch_value1, touch_value3, angC, motorB_UpSpeed, touchSensor1, touchSensor3);
pause(2);     % 2 sec pause to stable the sensor and smooth operation

% now read the distances for each position (A, B, and C) by sonic sensor
dist_b_down = readDistance(sonic2)*1000;    % dist*1000 to convert into mm 
disp('Distance at b = ');
disp(dist_b_down);
pause(2);     % 2 sec pause to stable the sensor and smooth operation
     
turnC(base_motor,  motorC_Speed, angC)
pause(2);     % 2 sec pause to stable the sensor and smooth operation
dist_c_down = readDistance(sonic2)*1000;   % dist*1000 to convert into mm 
disp('Distance at c = ');
disp(dist_c_down);

turn_a(base_motor, motorC_Speed, touch_value1, touchSensor1) 
pause(2);   % 2 sec pause to stable the sensor and smooth operation
dist_a_down = readDistance(sonic2)*1000;   % dist*1000 to convert into mm 
disp('Distance at a = ');
disp(dist_a_down);

gripper_operations("OPEN", gripper)     % Open jaw of gripper
pause(2);

% from inverse kinematics function reads the theta1 and theta2 angles
% for all positions based on the heaight measured by sonic sensor
[thta1a, thta2a] = inv_kine("A", l1, l2, l3, l4, h, dist_a_down);
[thta1b, thta2b] = inv_kine("B", l1, l2, l3, l4, h, dist_b_down);
[thta1c, thta2c] = inv_kine("C", l1, l2, l3, l4, h, dist_c_down);
disp('thta1a');
disp(thta1a);
disp('thta2a');
disp(thta2a);

disp('thta1b');
disp(thta1b);
disp('thta2b');
disp(thta2b);

disp('thta1c');
disp(thta1c);
disp('thta2c');
disp(thta2c);

% Task 5: Started
% pick the ball from station B and place it on station C.
home(motorC_Speed, base_motor, link_motor, touch_value1, touch_value3, angC, motorB_UpSpeed, touchSensor1, touchSensor3);
armDown(motorB_DownSpeed, link_motor, angB,thta2b,  touch_value3)
gripper_operations("CLOSE", gripper)    % gripper close pick the ball 
pause(1);     % pause for smoother operation
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)  % arm moves upward
disp('Ball picked up');     % display the string
turnC(base_motor,  motorC_Speed, angC)   % turn at C position
armDown(motorB_DownSpeed, link_motor, angB,thta2c,  touch_value3)       % arm moves downward
gripper_operations("OPEN", gripper)     %  gripper open, ball place
disp('Ball drop at c');     % display the string 
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
turn_b_fromC(base_motor, motorC_Speed, angC)   % return to Home Position after operation
pause(1);
disp('Reached at position B')
disp('Return at Home position: Task 5 Completed')
% Task 5: Completed

% Task 6: Started
% Pick the ball from Position C and place it on station A.
turnC(base_motor,  motorC_Speed, angC)     % Turn at C position
armDown(motorB_DownSpeed, link_motor, angB,thta2c,  touch_value3)      % arm moves downwards
gripper_operations("CLOSE", gripper)      % gripper close pick the ball
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
turn_a(base_motor, motorC_Speed, touch_value1, touchSensor1) 
armDown(motorB_DownSpeed, link_motor, angB,thta2a,  touch_value3)       % arm moves downwards
gripper_operations("OPEN", gripper)       %  gripper open, ball place
home(motorC_Speed, base_motor, link_motor, touch_value1, touch_value3, angC, motorB_UpSpeed, touchSensor1, touchSensor3)
pause(1);  % return to Home position
disp('Place the ball at position A and return to Home position: Task 6 Completed')
% Task 6: Completed

% Task 7: Started
% Pick the ball from station A and plcae it on position B.
turn_a(base_motor, motorC_Speed, touch_value1, touchSensor1)   % turn at position A
armDown(motorB_DownSpeed, link_motor, angB,thta2a,  touch_value3)      % arm moves downwards
gripper_operations("CLOSE", gripper)     % gripper close pick the ball
home(motorC_Speed, base_motor, link_motor, touch_value1, touch_value3, angC, motorB_UpSpeed, touchSensor1, touchSensor3)
armDown(motorB_DownSpeed, link_motor, angB,thta2b,  touch_value3)      % arm moves downwards
gripper_operations("OPEN", gripper)      %  gripper open, ball place
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
disp('Pick the ball fom A and place it on station B and return to Home position: Task 7 Completed');
% Task 7: Completed

% Task 8: Started
% Pick the ball from Position B and Place it on station A.
armDown(motorB_DownSpeed, link_motor, angB,thta2b,  touch_value3)      % arm moves downwards
gripper_operations("CLOSE", gripper)     % gripper close pick the ball
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)      % arm moves upward
turn_a(base_motor, motorC_Speed, touch_value1, touchSensor1) 
armDown(motorB_DownSpeed, link_motor, angB,thta2a,  touch_value3)      % arm moves downwards
gripper_operations("OPEN", gripper)      %  gripper open, ball place
home(motorC_Speed, base_motor, link_motor, touch_value1, touch_value3, angC, motorB_UpSpeed, touchSensor1, touchSensor3);
disp('Pick the ball from B and place it on A and return to Home: Task 8 Completed');
% Task 8: Completed

% Task 9: Started
% Pick the ball from station A and place it on station C.
turn_a(base_motor, motorC_Speed, touch_value1, touchSensor1)
armDown(motorB_DownSpeed, link_motor, angB,thta2a,  touch_value3)      % arm moves downwards
gripper_operations("CLOSE", gripper)      % gripper close pick the ball
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
turn_c_fromA(base_motor, motorC_Speed, angC)
armDown(motorB_DownSpeed, link_motor, angB,thta2c,  touch_value3)      % arm moves downwards
gripper_operations("OPEN", gripper)       %  gripper open, ball place
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
turn_b_fromC(base_motor, motorC_Speed, angC)
disp('Pick the ball from A and place it on C and return to Home position: Task 9 Completed')
% Task 9: Completed

% Task 10: Started
% Task 10: Pick the ball from station C and place it on position B.
turnC(base_motor,  motorC_Speed, angC) 
armDown(motorB_DownSpeed, link_motor, angB,thta2c,  touch_value3)      % arm moves downwards
gripper_operations("CLOSE", gripper)      % gripper close pick the ball
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
turn_b_fromC(base_motor, motorC_Speed, angC)
armDown(motorB_DownSpeed, link_motor, angB,thta2b,  touch_value3)      % arm moves downwards
gripper_operations("OPEN", gripper)       %  gripper open, ball place
arm_up(motorB_UpSpeed, link_motor, sonic2, touchSensor3, touch_value3)     % arm moves upward
disp('Pick the ball from C and place it on B and return to Home position: Task 10 Completed')
% Task 10: Completed



% all functions are defined below ****************************************
% Function to move the arm down by the theta2 angle 
function armDown(linkspeed, lmotor, angleB, thet_2, ts3)
    lmotor.Speed = linkspeed;    %  speed given to arm motor
    start(lmotor);               %  arm motor started
    resetRotation(lmotor);       %  rotation reset to zero for link motor
    angleB = readRotation(lmotor);
    disp(angleB);
    while (angleB <= thet_2)     % condition for arm to move downwards 
        angleB = readRotation(lmotor);
        %distance = readDistance(sonic2);
        disp(angleB);
    end
    lmotor.Speed = 0;     % arm motor stoped
    disp('ANGLE B');
    disp(angleB);         % shows current angle of arm link
    pause(2);
    ts3 = 0;  % upper sensor value set to zero
end


% Function to move to Home position from any position that means 
% Gripper is at Position B at top most position and gripper status open
function home(base_speed, baseM, linkM, ts1 ,ts3, base_angle, link_speed, touchSensor1, touchSensor3);
    % arm link moves up
    linkM.Speed = -1 *abs(link_speed);   % link motor started
    start(linkM);
    ts3 = readTouch(touchSensor3);       % touchvalue stored in ts3
    while(ts3 ~= 1)                      % condition for arm motor to stop
        ts3 = readTouch(touchSensor3);    
    end
    stop(linkM);                         % link motor stoped
    pause(1);
    baseM.Speed = base_speed;            % speed given to the base motor
    start(baseM)                         % base motor started
    ts1 = readTouch(touchSensor1);       % touchvalue stored in ts1
    while (ts1 ~= 1)                     % condition for base motor halt
        ts1 = readTouch(touchSensor1);
    end
    baseM.Speed = 0;                     % base motor speed set to zero
    pause(1)
    resetRotation(baseM)                 % reset rotation value of base motor
    base_angle = readRotation(baseM);    % rotation value asssigned to base angle
    disp(base_angle);
    
    baseM.Speed = -1*abs(base_speed);
    %start(base_motor)
    base_angle =   readRotation(baseM);
    while (base_angle >= -300)          % conditon for base motor to stop at 90 degree anticlockwise
        base_angle = readRotation(baseM);
    end
    baseM.Speed = 0;
    pause(1);
    resetRotation(baseM);               % rotation value of base motor reseted
    base_angle = readRotation(baseM);
    disp(base_angle)
    ts1 = 0;
end


%Function to operate the gripper in OPEN, CLSOE position
function gripper_operations(operation, gripperM)
    if (operation == "SET_INITIALLY")    % set the gripper initially
        resetRotation(gripperM);
        angA_ = readRotation(gripperM);
        start(gripperM);                 % started the gripper motor
        gripperM.Speed = 7;
        while ((angA_ <= 40) && (angA_ >= -40)) % conditon to stop gripper 
            angA_ = readRotation(gripperM);
        end
        gripperM.Speed = 0;
    elseif (operation == "OPEN")       % gripper in close operation
        angA_ = readRotation(gripperM);
        start(gripperM);               % gripper motor started
        gripperM.Speed = 15;
        while ((angA_ <= 70) && (angA_ >= -70))  % condition to stop gripper in open position
            angA_ = readRotation(gripperM);
        end
        gripperM.Speed = 0;
    elseif (operation == "CLOSE")   % gripper operation in close position
        angA_ = readRotation(gripperM);
        start(gripperM);
        gripperM.Speed = -60;
        while ((angA_ >= 25) || (angA_ <= -25))  % condition of gripper in close position
            angA_ = readRotation(gripperM);
        end
        gripperM.Speed = 0;         % gripper motor stopped
    else
        gripperM = 0; 
    end
end


% Functionn to move the arm upward
function arm_up(link_speed, linkM, sonic2, touchSensor3, ts3)
    linkM.Speed = -1 * abs(link_speed);   % negative speed given to arm motor
    start(linkM)         % arm motor started
    ts3 = readTouch(touchSensor3);
    while(ts3 ~=1)       % condition for arm motor to stop
        ts3 = readTouch(touchSensor3);
    end
    linkM.Speed = 0;    % arm motor stoped
    pause(2)
end


% Function to calculate theta 2 angle and theta 1 angle from HEIGHT
function [theta1, theta2] = inv_kine(Position, l1, l2, l3, l4, h, z1)
    
    theta1 = 0;
    theta2 = 0;
    gr1 = 3;   % gear ration for base motor
    gr2 = 5;   % gear ration for arm motor

    if (Position == "A")   % X, Y, and Z position for A station
        x = 111.7094;
        y = 0;
        z = z1;
    elseif (Position == "B") % X, Y, and Z position for B station
        x = 0;
        y = 111.7094;
        z = z1;
    elseif (Position == "C") % X, Y, and Z position for C station
        x = -111.7094;
        y = -5;
        z = z1;
    end
    
    theta1 = atand(y/x)*gr1*1.15;  % 1.15 is correction factor for the base motor
    disp('coordinates')
    disp(x)
    disp(y)
    disp(z)
    if (Position== "A")
        theta1 = atand(y/x)*gr1*1.15; % 1.15 is correction factor for the base motor

    elseif (Position == "B")
        theta1 = atand(y/x)*gr1*1.15;  % 1.15 is correction factor for the base motor

    elseif (Position == "C")
        theta1 = (atand(y/x)+180)*gr1*1.15;  % 1.15 is correction factor for the base motor
    end
    alpha = asind((z-h-l1-l2*sind(45)+l4)/l3);  % based on the station height we calculated alpha angle
    theta2 = (alpha + 45)*gr2*0.7;  % correction factor for ear ration / gear play = 0.7 depends on the robot
end


% Fucntion to turn the robot gripper at Position C 
function turnC (baseM, base_speed, angleC)
    
    baseM.Speed = -1 * abs(base_speed);   % base motor started
    resetRotation(baseM);     % reset the rotation
    angleC = readRotation(baseM);
    while (angleC >= -288*0.96)     % 0.96 is the correction factor due to play in plastic gears
        angleC = readRotation(baseM);
    end
    baseM.Speed = 0;     % base motor stopped
    pause(1);
end


% Function for the base motor to move from b position to C position
function turn_b_fromC(baseM, base_speed, angleC)
    baseM.Speed = abs(base_speed);   % base motor started
    resetRotation(baseM);
    angleC = readRotation(baseM);
    while (angleC <= 288)   % conditon to stop the motor when angleC is gretaer than 288
        angleC = readRotation(baseM);
    end
    baseM.Speed = 0;
    pause(1);

end


% Function to move the base motor at postion A
function turn_a(baseM, base_speed, ts1, touchSensor1)
    
    baseM.Speed = base_speed;    
    ts1 = readTouch(touchSensor1);
    while (ts1 ~= 1)
        ts1 = readTouch(touchSensor1);
    end
    baseM.Speed = 0;
    pause(1);
end


%  Function to turn gripper from station C to station A
function turn_c_fromA(baseM, base_speed, angleC)

    baseM.Speed = -1 * abs(base_speed);
    resetRotation(baseM);
    angleC = readRotation(baseM);
    while (angleC >= -600*0.97)  
        angleC = readRotation(baseM);
    end
    baseM.Speed = 0;
    pause(1);
end