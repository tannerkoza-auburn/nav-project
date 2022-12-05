%% GNSS/INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V2-Log2";
gpsSamplingFreq = 20; % downsampled gps frequency
gpsVelNormThreshold = 5; % for dynamic attitude initialization
gpsCourseBufferLength = 50; % samples

%% LOAD DATA

fprintf('nav-project: loading and calculating additional data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% CONSTANTS

insSamplingFreq = 200;

%% FILTER INITIALIZATION

% Time
numSamplesIMU = length(data.imu.timeDuration);
gpsSampleInterval = insSamplingFreq / gpsSamplingFreq;
insTime = data.imu.timeEpoch;
gpsTime = data.gps.timeEpoch(1:gpsSampleInterval:end);

% Attitude
insEulerAnglesLog = zeros(numSamplesIMU, 3);
% Cbn = eye(3);
Cbn = [0.6428 0.7660 0; ...
    -0.7660 0.6428 0; ...
    0 0 1.0000]'; % unaligned with nav-frame

% Cbn = [-0.8788    0.4772         0;
%    -0.4772   -0.8788         0;
%          0         0    1.0000];
q = [0; 0; 0; 1];

% Csb = [-1.0000   -0.0000    0.0001;
%     0.0000   -1.0000   -0.0001;
%     0.0001   -0.0001    1.0000]';

Csb = eye(3);

% Velocity
insVelocityNED = [0 0 0];
insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = data.gps.LLA(1, :); % initial position
mapOriginLLA = [42.294319 -83.223275 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
gpsPositionLLA = data.gps.LLA(1:gpsSampleInterval:end, :);

posNED = [0; 0; 0];

for i = 50000:numSamplesIMU

    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    wBI_B = Csb * data.imu.angular_velocity(i, :)';
    fBI_B = Csb * data.imu.linear_acceleration(i, :)';

    gravityNED = [0; 0; 0];
    %         gravityNED = gravity(insPositionLLA(1),insPositionLLA(3))';
    omegaEI_N = earth_rate(insPositionLLA(1));
    omegaNE_N = transport_rate(insPositionLLA(1), insVelocityNED(1), insVelocityNED(2), insPositionLLA(3));
    [~, Cbn, insEulerAngles] = att_update(wBI_B, Cbn, q, omegaEI_N, omegaNE_N, dt, 'dcm');

    fBI_N = Cbn * fBI_B;

    insVelocityNED = vel_update(fBI_B, insVelocityNED, omegaEI_N, omegaNE_N, gravityNED, dt);

    insPositionLLA = pos_update(insPositionLLA, insVelocityNED, dt);

    % Logging
    insPositionLLALog(i, :) = insPositionLLA;
    insEulerAnglesLog(i, :) = rad2deg(insEulerAngles);
    insVelocityNEDLog(i, :) = insVelocityNED';

end

%%

% Colors
blue = [0, 0.4470, 0.7410];
orange = [0.8500, 0.3250, 0.0980];
green = [0.4660, 0.6740, 0.1880];
yellow = [0.9290, 0.6940, 0.1250];
lightBlue = [0.3010, 0.7450, 0.9330];

% Font Sizes
fontTitle = 25;
fontLabel = 15;
fontTick = 10;
fontLegend = 10;

% Line Width
lineWidth = 1.75;

% Marker Size
markerSize = 10;

% Attitude
figure("Name", "Attitude: Euler Angles")
subplot(3, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 1), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 3), '.')
hold on

l3 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3 = title('Attitude: Yaw');
x3 = xlabel('time [s]');
y3 = ylabel('yaw [deg]');
ax3 = gca;

grid
set(t3, 'FontSize', fontTitle);
set(x3, 'FontSize', fontLabel);
set(y3, 'FontSize', fontLabel);
set(l3, 'FontSize', fontLegend);

subplot(3, 1, 2)
plot(data.truth.timeDuration, data.truth.euler(:, 2), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 2), '.')
hold on

l4 = legend('Truth', 'INS', 'Location', 'SouthEast');
t4 = title('Attitude: Pitch');
x4 = xlabel('time [s]');
y4 = ylabel('pitch [deg]');
ax4 = gca;

grid
ax4.YLim = ax3.YLim;
set(t4, 'FontSize', fontTitle);
set(x4, 'FontSize', fontLabel);
set(y4, 'FontSize', fontLabel);
set(l4, 'FontSize', fontLegend);

subplot(3, 1, 3)
plot(data.truth.timeDuration, data.truth.euler(:, 3), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 1), '.')
hold on

l5 = legend('Truth', 'INS', 'Location', 'SouthEast');
t5 = title('Attitude: Roll');
x5 = xlabel('time [s]');
y5 = ylabel('roll [deg]');
ax5 = gca;

grid
ax5.YLim = ax3.YLim;
set(t5, 'FontSize', fontTitle);
set(x5, 'FontSize', fontLabel);
set(y5, 'FontSize', fontLabel);
set(l5, 'FontSize', fontLegend);

% Velocity
figure('Name', 'NED Velocity')
subplot(3, 1, 1)
plot(data.raw_velocity.timeEpoch, data.raw_velocity.NED(:, 1), '.k')
hold on
plot(data.imu.timeEpoch, insVelocityNEDLog(:, 1))

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('Velocity: North');
x6 = xlabel('time [s]');
y6 = ylabel('velocity [m/s]');
ax6 = gca;
ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(3, 1, 2)
plot(data.raw_velocity.timeEpoch, data.raw_velocity.NED(:, 2), '.k')
hold on
plot(data.imu.timeEpoch, insVelocityNEDLog(:, 2))

l7 = legend('Truth', 'INS', 'Location', 'SouthEast');
t7 = title('Velocity: East');
x7 = xlabel('time [s]');
y7 = ylabel('velocity [m/s]');
ax7 = gca;

grid
ax7.YLim = ax6.YLim;
set(t7, 'FontSize', fontTitle);
set(x7, 'FontSize', fontLabel);
set(y7, 'FontSize', fontLabel);
set(l7, 'FontSize', fontLegend);

subplot(3, 1, 3)
plot(data.raw_velocity.timeEpoch, data.raw_velocity.NED(:, 3), '.k')
hold on
plot(data.imu.timeEpoch, insVelocityNEDLog(:, 3))

l8 = legend('Truth', 'INS', 'Location', 'SouthEast');
t8 = title('Velocity: Down');
x8 = xlabel('time [s]');
y8 = ylabel('velocity [m/s]');
ax8 = gca;

grid
ax8.YLim = ax6.YLim;
set(t8, 'FontSize', fontTitle);
set(x8, 'FontSize', fontLabel);
set(y8, 'FontSize', fontLabel);
set(l8, 'FontSize', fontLegend);