%%
clear
clc
close all 

%% LOAD DATA

fprintf('nav-project: loading sensor and truth data...\n')


%% PLOT PARAMETERS

blue    = [0, 0.4470, 0.7410];
orange  = [0.8500, 0.3250, 0.0980];
green   = [0.4660, 0.6740, 0.1880];
yellow  = [0.9290, 0.6940, 0.1250];
light_blue = [0.3010, 0.7450, 0.9330];

font_title  = 25;
font_label  = 15;
font_tick   = 10;
font_legend = 10;

lw = 1.75;

ms = 10;


%% Load Data

    rawData = load('fordav_sample_data');

%% Extract Necessary Values

    initLat = 42.294319;
    initLon =  -83.223275;
    initAlt = 0;

    xTruthNED = rawData.pose_ground_truth.pose_position_x;
    yTruthNED = rawData.pose_ground_truth.pose_position_y;
    zTruthNED = rawData.pose_ground_truth.pose_position_z;

%% Calculate New Values

    truthECEF = ned2ecef(...
        [xTruthNED yTruthNED zTruthNED],...
        [deg2rad(initLat) deg2rad(initLon) initAlt]);

  truthLLA = rad2deg(ecef2llh(truthECEF));

 gpsECEF = llh2ecef([deg2rad(rawData.gps.latitude) deg2rad(rawData.gps.longitude) rawData.gps.altitude]);
 gpsNED = ecef2ned(gpsECEF, [deg2rad(initLat) deg2rad(initLon) initAlt]);

%% PLOT SENSOR DATA

% fprintf('nav-project: plotting sensor data...\n')

% t = imu.Time - imu.Time(1);
% 
% figure('Name','Raw IMU Data')
% 
% subplot(2,1,1)
% plot(t, imu.angular_velocity_x, '.')
% hold on
% plot(t, imu.angular_velocity_y, '.')
% plot(t, imu.angular_velocity_z, '.')
% title('Angular Velocities')
% 
% subplot(2,1,2)
% plot(t, imu.linear_acceleration_x, '.')
% hold on
% plot(t, imu.linear_acceleration_y, '.')
% plot(t, imu.linear_acceleration_z, '.')
% title('Linear Accelerations')


%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')

% Global Frame Trajectory
figure("Name","Local Frame Trajectory")
plot(rawData.pose_ground_truth.pose_position_x, rawData.pose_ground_truth.pose_position_y, ...
    '--k')
hold on
plot(gpsNED(:,1), gpsNED(:,2))
hold off

l1 = legend('Truth', 'GPS', ...
    'Location', 'SouthEast');
t1 = title('2D Trajectory: NED');
x1 = xlabel('x [m]');
y1 = ylabel('y [m]');

grid
set(t1,'FontSize', font_title);
set(x1,'FontSize', font_label);
set(y1,'FontSize', font_label);
set(l1,'FontSize', font_legend);
set(gca, 'YTickMode', 'auto', 'FontSize', font_tick);
axis equal
axis padded

% 2D Trajectory
figure("Name","Geoplot Trajectory")

geoplot(truthLLA(:,1), truthLLA(:,2))
hold on
geoplot(rawData.gps.latitude, rawData.gps.longitude)

geobasemap satellite

