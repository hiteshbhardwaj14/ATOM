clear all;
close all;
clc;

global s1 data count exec_t_start;

data = zeros(50000, 60);
count = 1;
exec_t_start = tic;

s1 = serialport('COM4', 57600);
% configureTerminator(s1, "CR", 10);

% fclose(instrfind);
% fopen(s1);

c = natnet;
c.ConnectionType = 'Multicast';
c.connect;
% c.IsReporting = 1;
c.addlistener(1, 'ATOM_matlabcode');  % rolling_v1

c.enable(0);


%%

%% For offline debug

% for i = 0:100
%     RBnum = 1;
%     
%     evnt.data.RigidBodies(RBnum).x = 10;
%     evnt.data.RigidBodies(RBnum).y = 0;
%     evnt.data.RigidBodies(RBnum).z = 1;
%     q = eul2quat([0, 0, -7*i/180*pi], 'ZYX');
%     evnt.data.RigidBodies(RBnum).qw = q(1);
%     evnt.data.RigidBodies(RBnum).qx = q(2);
%     evnt.data.RigidBodies(RBnum).qy = q(3);
%     evnt.data.RigidBodies(RBnum).qz = q(4);
%     
%     record_callback(NaN, evnt);
% end
% 
% colormap parula
% eul = quat2eul(data(:,8:11), 'XYZ');
% scatter(eul(1:count,3)/pi*180, data(1:count,end), 10, linspace(1, 10, count), 'filled');

% [eul(1:100,3)/pi*180 data(1:100,end)]

%% To disable the listener
%%% c.disconnect;