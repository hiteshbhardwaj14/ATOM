function ch = ATOM_matlabcode(~, evnt)
% Corrected the gains for 4s battery model
global exec_t_start off_dir s1 data count...
    Kp_slm Kd_slm phi_s C omega_fac base_thrust;

persistent t_start t_old ep_old q_old sum_eHeading sum_epx sum_epy p_old...
            init_roll thr_rol_mode start_roll_fn t_initRoll t_finRoll t_Rolltime t_eHeading;

if count == length(data)
    return
end

%% Parameters initialization

exec_t = toc(exec_t_start); % execution time

if isempty(t_start)
    t_start = tic;
    t_old = 0;
    p_old = [0; 0; 0];
    ep_old = [0; 0; 0];
    q_old = [0, 0, 0, 0];
    
    sum_eHeading = zeros(1,20);
    sum_epx = zeros(1,30);
    sum_epy = zeros(1,30);
    init_roll = 0;
    thr_rol_mode = 0;
    t_initRoll = -1;
    t_finRoll = 0;
    t_Rolltime = 0;
    t_eHeading = -1;
    start_roll_fn = 0;
end

% t = evnt.data.fTimestamp;
t = toc(t_start); % execution time
dt = t - t_old;
if dt > 0.04
    dt = 0.04;
end

% base_thrust = 1700;
off_dir = pi/2-pi/4;%(for CCW)  %-pi/4;%(for CW)
phi_s = [0.1 0.1 0.5]';
C = [0.7 0.7 40]';

base_thrust = 1720;
omega_fac = 0.5;
Ki_ep = 0.02;             
Kp_slm = [1.5 1.5 3]'; 
Kd_slm = [0.15 0.15 1]';   
man_diff = 115;



%% Trajectory
% p_d = [0 0 1];

% p_d = [1 1 1];
% base_time = 5;
% interval = 20;
% if (t <= base_time)
%     p_d = [1 1 1];
% end
% if (t > base_time) && (t <= base_time+(interval))
%     p_d = [-1 1 1];
% end
% if (t > base_time+(interval)) && (t <= base_time+(2*interval))
%     p_d = [-1 -1 1];
% end
% if (t > base_time+(2*interval)) && (t <= base_time+(3*interval))
%     p_d = [1 -1 1];
% end
% if (t > base_time+(3*interval)) && (t <= base_time+(4*interval))
%     p_d = [1 1 1];
% end 
% if (t > base_time+(4*interval)) && (t <= base_time+(5*interval))
%     p_d = [-1 1 1];
% end
% if (t > base_time+(5*interval)) && (t <= base_time+(6*interval))
%     p_d = [-1 -1 1];
% end
% if (t > base_time+(6*interval)) && (t <= base_time+(7*interval))
%     p_d = [1 -1 1];
% end
% if (t > base_time+(7*interval)) && (t <= base_time+(8*interval))
%     p_d = [1 1 1];
% end
% 
% %%%%%%%%%% Heading test %%%%%%%%%%%%%%%
p_d = [-2 0 1];
base_time = 10;
interval = 10;
if (t <= base_time)
    p_d = [-2 0 1];
end
if (t > base_time) && (t <= base_time+(interval))
    p_d = [-2 -2 1];
end
if (t > base_time+(interval)) && (t <= base_time+(2*interval))
    p_d = [2 -2 1];
end
if (t > base_time+(2*interval)) && (t <= base_time+(3*interval))
    p_d = [-2 0 1];
end
if (t > base_time+(3*interval)) && (t <= base_time+(4*interval))
    p_d = [2 0 1];
end 

%% Get Optitrack data..........................

rb_mono1 = 1;

mono_pose(1) = evnt.data.RigidBodies(rb_mono1).x;
mono_pose(2) = evnt.data.RigidBodies(rb_mono1).y;
mono_pose(3) = evnt.data.RigidBodies(rb_mono1).z;
mono_pose(4) = evnt.data.RigidBodies(rb_mono1).qw;
mono_pose(5) = evnt.data.RigidBodies(rb_mono1).qx;
mono_pose(6) = evnt.data.RigidBodies(rb_mono1).qy;
mono_pose(7) = evnt.data.RigidBodies(rb_mono1).qz;
% mono_pose(8) = evnt.data.fTimestamp;

%% Get position and attitude

% t = evnt.data.fTimestamp;
% % t = toc(t_start); % execution time
% dt = t - t_old;

p = mono_pose(1:3);
quat = mono_pose(4:7);
q_0 = quat(1);
q_1 = quat(2);
q_2 = quat(3);
q_3 = quat(4);

eul = quat2eul(quat, 'XYZ');
% rol = eul(1);
% pit = eul(2);
yaw = eul(3);

dq = (quat - q_old)/dt;
W = [-q_1, q_0, -q_3, q_2; -q_2, q_3, q_0, -q_1; -q_3, -q_2, q_1, q_0];
Wb = [-q_1, q_0, q_3, -q_2; -q_2, -q_3, q_0, q_1; -q_3, q_2, -q_1, q_0];
Omega_W = 2 * W * dq';
Omega_Wb = 2 * Wb * dq';
%..................................................

buf_W = 20;
if count > buf_W
    filt_Wx = mean(data(count-buf_W:count,15));
    filt_Wy = mean(data(count-buf_W:count,16));
    filt_Wz = mean(data(count-buf_W:count,17));
else
    filt_Wx = Omega_W(1);
    filt_Wy = Omega_W(2);
    filt_Wz = Omega_W(3);
end
filt_W = [filt_Wx filt_Wy filt_Wz];

%% Mono controller ......................

ep = p_d - p; % position error
ev = (ep - ep_old)/dt;
sum_epx = [sum_epx(2:end), ep(1)];
sum_epy = [sum_epy(2:end), ep(2)];

sx = Kp_slm(1) * ep(1) + Kd_slm(1) * ev(1) + Ki_ep * sum(sum_epx);
sy = Kp_slm(2) * ep(2) + Kd_slm(2) * ev(2) + Ki_ep * sum(sum_epy);
sz = Kp_slm(3) * ep(3) + Kd_slm(3) * ev(3);

if (abs(sx) > phi_s(1))
    sat_sx = sign(sx);
else
    sat_sx = sx/phi_s(1);
end
if (abs(sy) > phi_s(2))
    sat_sy = sign(sy);
else
    sat_sy = sy/phi_s(2);
end
if (abs(sz) > phi_s(3))
    sat_sz = sign(sz);
else
    sat_sz = sz/phi_s(3);
end
 
ux = sat_sx * C(1);
uy = sat_sy * C(2);
ux = ux - omega_fac * filt_W(1);
uy = uy - omega_fac * filt_W(2);
uz = (sat_sz * C(3)) + base_thrust;

ux = min(max(ux, -1), 1); 
uy = min(max(uy, -1), 1);


%% CYCLIC CONTROL PART ..........................

T_amp = 50*sqrt(ux^2 + uy^2);
com_dir = atan2(uy, -ux);
condition1 = sin(yaw + com_dir + off_dir);

if condition1 > 0
    thr_m1 = uz + T_amp;
else
    thr_m1 = uz - T_amp;
end

thr_m2 = uz - man_diff;

thr_m1 = min(max(thr_m1, 1500.00), 2000.00);
thr_m2 = min(max(thr_m2, 1500.00), 2000.00);

%% Obtaining roll and rotation data 
dcm_world = quat2dcm(quat);
quat_body = quatconj(quat);
dcm_body = quat2dcm(quat_body);

x_world = dcm_world(:,1);
y_world = dcm_world(:,2);
z_world = dcm_world(:,3);

x_body = dcm_body(:,1);
y_body = dcm_body(:,2);
z_body = dcm_body(:,3);

currentRoll = -atan2(z_world(2),z_world(1));
currentRoll = round(currentRoll,2);
currentRoll_off = currentRoll - pi/2;

if (currentRoll_off > pi)
    currentRoll_off = currentRoll_off - (2*pi);
end
if (currentRoll_off < -pi)
    currentRoll_off = currentRoll_off + (2*pi);
end

currentHeading = atan2(-z_body(2),-z_body(1));
if currentHeading < 0
    currentHeading = (2*pi) + currentHeading;
end

%% Rolling and turning PART 
buf_Rolloff = 50;
if count > buf_Rolloff
    filt_currentRoll_off = mean(data(count-buf_Rolloff:count,52));
else
    filt_currentRoll_off = currentRoll_off;
end
% ------------------------------------------------------------------------
kp_stop = -5;
kd_stop = 20; 
ki_head_w_roll = 14;
head_IB = 50; %200;
ki_head = 1; %0.7;
head_IB_dir = 130; %200;
des_roll_ang = -0.03; 
rol_thr_amp = 65;

if (t <= 20)
    head_IB_dir = 120;
end
    
% ------------------------------------------------------------------------
v = (p - p_old)/dt;
ePosition = sqrt((p_d(1)-p(1))^2 + (p_d(2)-p(2))^2); 
des_roll_heading = atan2((p(1)-p_d(1)),(p_d(2)-p(2)));
if des_roll_heading < 0
    des_roll_heading = (2*pi) + des_roll_heading;
end
if (abs(ePosition) < 1)
    des_roll_heading = currentHeading;
end
if (des_roll_heading > 0 && des_roll_heading < pi)
    eHeading = des_roll_heading - currentHeading;
    if (currentHeading > (pi+des_roll_heading))
        eHeading = (2*pi - currentHeading) + des_roll_heading;
    end
end
if (des_roll_heading > pi && des_roll_heading < 2*pi)
    eHeading = des_roll_heading - currentHeading;
    if (currentHeading < (des_roll_heading-pi))
        eHeading = des_roll_heading - 2*pi - currentHeading;
    end
end

eHeading = rad2deg(eHeading);
sum_eHeading = [sum_eHeading(2:end), eHeading];
sum_eHead = sum(sum_eHeading);
sum_eHead = min(max(sum_eHead, -head_IB), head_IB);
sum_eHead_dir = sum(sum_eHeading);
sum_eHead_dir = min(max(sum_eHead_dir, -head_IB_dir), head_IB_dir);

str_dir = (sum_eHead_dir * ki_head);  


% ---- ALGORITHM ----------------------------------------------------------

if (abs(ePosition) > 0.2)
% -----------------------------------------
    if (abs(eHeading) < 5)
        str_dir = 0;
        if (t_eHeading == -1)
            t_eHeading = t;
        end
        if (t - t_eHeading  < 2)
            thr_rol_mode = 0;
            start_roll_fn = 0;
        else
        % -----------------------------------------
            if ((init_roll == 0) && (currentRoll_off > -0.5) && (currentRoll_off < 0.5))
                start_roll_fn = 1;
                thr_rol_mode = 0;
            else
                start_roll_fn = 0;
                thr_rol_mode = 1;
            end
        % -----------------------------------------
        end
    else
        start_roll_fn = 0;
        init_roll = 0;
        t_initRoll = -1;
        t_eHeading = -1;
        thr_rol_mode = 0;
    end
% -----------------------------------------
else
    start_roll_fn = 0;
    init_roll = 0;
    t_initRoll = -1;
    t_eHeading = -1;
    thr_rol_mode = 0;
    if (abs(eHeading) < 5)
        str_dir = 0;
    end
end


if (thr_rol_mode > 0)         % Forward motion
    if ((currentRoll_off > -pi) && (currentRoll_off < 0))
        thrust_roll1 = 1500 - rol_thr_amp*sin(currentRoll_off);       
        thrust_roll2 = 1500 - rol_thr_amp*sin(currentRoll_off);      
    else
        thrust_roll1 = 1500;
        thrust_roll2 = 1500;
    end
    
elseif (thr_rol_mode < 0)    % Backward motion
    if ((currentRoll_off > 0) && (currentRoll_off < pi))
        thrust_roll1 = 1500 + rol_thr_amp*cos(currentRoll_off);        
        thrust_roll2 = 1500 + rol_thr_amp*cos(currentRoll_off);         
    else
        thrust_roll1 = 1500;
        thrust_roll2 = 1500;
    end
else                           % STOP pendulum motion
    if ((currentRoll_off > -(pi/2)) && (currentRoll_off < (pi/2)))
        thrust_roll1 = 1500 + kp_stop*(des_roll_ang - filt_currentRoll_off) + kd_stop*(0 - Omega_Wb(3)); 
        thrust_roll2 = 1500 + kp_stop*(des_roll_ang - filt_currentRoll_off) + kd_stop*(0 - Omega_Wb(3));  
    else
        thrust_roll1 = 1500;
        thrust_roll2 = 1500;
    end   
end

if ((thrust_roll1 > 1470) && (thrust_roll1 < 1520))
    thrust_roll1 = 1500;
end
if ((thrust_roll2 > 1470) && (thrust_roll2 < 1520))
    thrust_roll2 = 1500;
end
% -------------------------------------------------------------------------
if (start_roll_fn == 1)                     % STARTs ROLL FROM STATIONARY POSITION
    if (t_initRoll == -1)
        t_initRoll = t;
        t_finRoll = t+1.1;
        t_Rolltime = 0;
    end
    t_initRoll = 1;
    if (t < t_finRoll)
        str_dir = 0;
        initial_ramp = 1500;
        final_ramp = 1600; %1615;
        thrust_roll1 = initial_ramp + (final_ramp - initial_ramp)*t_Rolltime^2;
        thrust_roll2 = initial_ramp + (final_ramp - initial_ramp)*t_Rolltime^2;
        thrust_roll1 = min(max(thrust_roll1, 1500), 1650);
        thrust_roll2 = min(max(thrust_roll2, 1500), 1650);
        t_Rolltime = t_Rolltime+dt;
    else
        init_roll = 1;
        start_roll_fn = 0;
    end
else
    t_initRoll = -1;
end
% -------------------------------------------------------------------------
m1_rol_mode = thrust_roll1 + str_dir;
m2_rol_mode = thrust_roll2 - str_dir;
m1_rol_mode = min(max(m1_rol_mode, 1000.0), 2000.0);
m2_rol_mode = min(max(m2_rol_mode, 1000.0), 2000.0);

%% Send control signal..........................
ch(1) = 1000;
ch(2) = m1_rol_mode; %m1_rol_mode; %thr_m1;
ch(3) = m2_rol_mode; %m2_rol_mode; %thr_m2;
ch(4) = 1500;
ch(5) = 1000;


% formatSpec = '%.2f';
sendString = strcat(int2str(ch(1)), ",",  int2str(ch(2)), ",", int2str(ch(3)), ",",int2str(ch(4)), ",",int2str(ch(5)));

% if ~isempty(s1)
%     writeline(s1, sendString);
% end


%% Save data

data(count, :) = [exec_t mono_pose yaw ch Omega_W'...
                    filt_W ux uy uz eul p_d t...
                    quat_body x_body' y_body' z_body' Omega_Wb'...
                    ePosition thrust_roll1 thrust_roll2 eHeading des_roll_heading currentRoll_off str_dir currentHeading...
                    thr_rol_mode filt_currentRoll_off thr_m1 thr_m2 m1_rol_mode m2_rol_mode];
count = count + 1;

t_old = t;
ep_old = ep;
q_old = quat;
p_old = p;

plot(t,currentRoll, 'r*')
hold on
plot(t,dm_m2, 'g*')

%% Debug

disp([num2str(t),'  ', num2str(des_roll_heading),'  ', num2str(currentHeading)...
        ,'  ', num2str(0),'  ', num2str(eHeading)]);

end
