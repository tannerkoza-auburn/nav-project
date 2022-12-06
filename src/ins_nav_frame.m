%% INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log3";
gpsSamplingFreq = 100; % downsampled gps frequency
gpsVelNormThreshold = 10; % for dynamic attitude initialization
bufferLength = 100; % samples

%% LOAD DATA

fprintf('nav-project: loading and calculating additional data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% CONSTANTS

insSamplingFreq = 200;

%% INS INITIALIZATION

% Time
numSamplesIMU = length(data.imu.timeDuration);
gpsSampleInterval = insSamplingFreq / gpsSamplingFreq;

insTime = data.imu.timeEpoch;
gpsTime = data.gps.timeEpoch(1:gpsSampleInterval:end);

% Alignment
gpsCourseBuffer = zeros(bufferLength, 1);
bufferIdx = 1;

% Attitude
insEulerAngles = [0 0 0];

Cbn = qua2dcm(data.truth.quaternion(1,:));
% Cbn = [0.6428 0.7660 0; ...
%     -0.7660 0.6428 0; ...
%     0 0 1.0000]'; % unaligned with nav-frame

% Cbn = [  0.6887   -0.7251         0;
%     0.7251    0.6887         0;
%          0         0    1.0000];


insEulerAnglesLog = zeros(numSamplesIMU, 3);

% Velocity
insVelocityNED = [0; 0; 0];

insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = data.gps.LLA(1,:); % initial position
gpsPositionLLA = data.gps.LLA(1:gpsSampleInterval:end, :);
mapOriginLLA = [deg2rad(42.294319) deg2rad(-83.223275) 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
insPositionLLALog(1,:) = insPositionLLA;

% Status Booleans
insAligned = 0;
gpsMeasStatus = 0;

%% INS MECHANIZATION

for i = 2:numSamplesIMU

    % Extract Sensor Data
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    wBI_B = data.imu.angular_velocity(i, :)';
    fBI_B = data.imu.linear_acceleration(i, :)';

%     % Dynamic Alignment
%     % check if filter is aligned and two gps positions are available
%     
%     gpsMeasStatus = ~isempty(gpsIdx);
% 
%     if ~insAligned && gpsMeasStatus
% 
%         % calculate gps course and velocity norm
%         [gpsCurrentCourse, gpsVelNorm, insVelocityNED] = gnssCourse( ...
%             gpsPositionLLA(gpsIdx, :), gpsPositionLLA(gpsIdx-1, :), ...
%             dt, mapOriginLLA);
%         insPositionLLA = gpsPositionLLA(gpsIdx, :); % update insPositionLLALog 
% 
%         if all(gpsCourseBuffer)
%             meanGPSCourse = mean(rmoutliers(gpsCourseBuffer));
%             Cbn = navtools.genDCM(meanGPSCourse - 0.0395, 'z', 'rads')'; % initial attitude (heading)
% 
%             insPositionLLA = gpsPositionLLA(gpsIdx, :); % initalize ins position
%             alignIndexIMU = i;
%             insAligned = 1;
% 
%             fprintf('nav-project: filter is aligned...\n')
% 
%         elseif gpsVelNorm > gpsVelNormThreshold
%             gpsCourseBuffer(bufferIdx) = gpsCurrentCourse;
%             bufferIdx = bufferIdx + 1;
%         end
% 
%     elseif insAligned
        % Calculate Angular Rates to Remove
        % NOTE: gravity is already removed from the specific force measurements
        omegaEI_N = earthRate(insPositionLLA(1));
        omegaNE_N = transportRate(insPositionLLA, insVelocityNED);
    
        % Attitude Update
        [Cbn, insEulerAngles] = attitudeUpdate(wBI_B, Cbn, omegaEI_N,...
            omegaNE_N, dt, 'dcm', 'precise');
    
        % Velocity Update
        fBI_N = Cbn * fBI_B;
        insVelocityNED = velocityUpdate(fBI_N, insVelocityNED,...
            omegaEI_N, omegaNE_N, dt);
    
        % Position Update
        insPositionLLA = positionUpdate(insPositionLLA, insVelocityNED, dt);

%     end

    % Logging
    insPositionLLALog(i, :) = insPositionLLA;
    insEulerAnglesLog(i, :) = insEulerAngles;
    insVelocityNEDLog(i, :) = insVelocityNED';

end

%% Error Calculations

eulerInterp = interp1(data.truth.timeEpoch, data.truth.euler, data.imu.timeEpoch);
velInterp = interp1(data.raw_velocity.timeEpoch, data.raw_velocity.NED, data.imu.timeEpoch);
% sqrt(mean((state_truth - pdr).^2))

posNED = cumsum(diff(ecef2ned(llh2ecef(insPositionLLALog), mapOriginLLA)));
posNED = [zeros(1,3); posNED];
truthNED = cumsum(diff(data.truth.NED));
truthNED = interp1(data.truth.timeEpoch(2:end), truthNED, data.imu.timeEpoch(2:end));
truthNED = [zeros(1,3); truthNED];

RMSE = sqrt(mean((truthNED - posNED).^2))

%% Plotting

% Colors
green = [0, 0.4470, 0.7410];
orange = [0.8500, 0.3250, 0.0980];
green = [0.4660, 0.6740, 0.1880];
yellow = [0.9290, 0.6940, 0.1250];
green = [0.3010, 0.7450, 0.9330];

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
% Yaw
figure("Name", "Yaw")
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 1), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 3), '.', 'Color', green)
hold on

l3 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3 = title('Yaw Comparison');
x3 = xlabel('time [s]');
y3 = ylabel('yaw [deg]');
ax3 = gca;

grid
set(t3, 'FontSize', fontTitle);
set(x3, 'FontSize', fontLabel);
set(y3, 'FontSize', fontLabel);
set(l3, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insEulerAnglesLog(:,3) - eulerInterp(:,1), '.r')

t3e = title('Yaw Error');
x3e = xlabel('time [s]');
y3e = ylabel('yaw error [deg]');
ax3e = gca;
ax3e.YLim = [-5 5];

grid
set(t3e, 'FontSize', fontTitle);
set(x3e, 'FontSize', fontLabel);
set(y3e, 'FontSize', fontLabel);

% Pitch
figure("Name", "Pitch")
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 2), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 2), '.', 'Color', green)
hold on

l3 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3 = title('Pitch Comparison');
x3 = xlabel('time [s]');
y3 = ylabel('pitch [deg]');
ax3 = gca;

grid
set(t3, 'FontSize', fontTitle);
set(x3, 'FontSize', fontLabel);
set(y3, 'FontSize', fontLabel);
set(l3, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insEulerAnglesLog(:,2) - eulerInterp(:,2), '.r')

t3e = title('Pitch Error');
x3e = xlabel('time [s]');
y3e = ylabel('pitch error [deg]');
ax3e = gca;
ax3e.YLim = [-5 5];

grid
set(t3e, 'FontSize', fontTitle);
set(x3e, 'FontSize', fontLabel);
set(y3e, 'FontSize', fontLabel);

% Roll
figure("Name", "Roll")
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 3), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 1), '.', 'Color', green)
hold on

l3 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3 = title('Roll Comparison');
x3 = xlabel('time [s]');
y3 = ylabel('roll [deg]');
ax3 = gca;

grid
set(t3, 'FontSize', fontTitle);
set(x3, 'FontSize', fontLabel);
set(y3, 'FontSize', fontLabel);
set(l3, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insEulerAnglesLog(:,3) - eulerInterp(:,1), '.r')

t3e = title('Roll Error');
x3e = xlabel('time [s]');
y3e = ylabel('roll error [deg]');
ax3e = gca;
ax3e.YLim = [-5 5];

grid
set(t3e, 'FontSize', fontTitle);
set(x3e, 'FontSize', fontLabel);
set(y3e, 'FontSize', fontLabel);

% Velocity
% North
figure('Name', 'North Velocity')
subplot(2, 1, 1)
plot(data.imu.timeDuration, velInterp(:, 1), '.k')
hold on
plot(data.imu.timeDuration, insVelocityNEDLog(:, 1),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('North Velocity Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('velocity [m/s]');
ax6 = gca;
ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insVelocityNEDLog(:,1)- velInterp(:, 1), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('North Velocity Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('velocity error [m/s]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% East
figure('Name', 'East Velocity')
subplot(2, 1, 1)
plot(data.imu.timeDuration, velInterp(:, 2), '.k')
hold on
plot(data.imu.timeDuration, insVelocityNEDLog(:, 2),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('East Velocity Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('velocity [m/s]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insVelocityNEDLog(:,2)- velInterp(:, 2), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('East Velocity Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('velocity error [m/s]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% Down
figure('Name', 'Down Velocity')
subplot(2, 1, 1)
plot(data.imu.timeDuration, velInterp(:, 3), '.k')
hold on
plot(data.imu.timeDuration, insVelocityNEDLog(:, 3),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('Down Velocity Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('velocity [m/s]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, insVelocityNEDLog(:,3)- velInterp(:, 3), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('Down Velocity Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('velocity error [m/s]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% Position
% North
figure('Name', 'North Position')
subplot(2, 1, 1)
plot(data.imu.timeDuration, truthNED(:, 1), '.k')
hold on
plot(data.imu.timeDuration, posNED(:, 1),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('North Velocity Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('position [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, posNED(:,1)- truthNED(:, 1), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('North Position Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('position error [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% East
figure('Name', 'East Position')
subplot(2, 1, 1)
plot(data.imu.timeDuration, truthNED(:, 2), '.k')
hold on
plot(data.imu.timeDuration, posNED(:, 2),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('East Position Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('position [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, posNED(:,2)- truthNED(:, 2), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('East Position Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('position error [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% Down
figure('Name', 'Down Position')
subplot(2, 1, 1)
plot(data.imu.timeDuration, truthNED(:, 3), '.k')
hold on
plot(data.imu.timeDuration, posNED(:, 3),'.','Color', green)

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('Down Position Comparison');
x6 = xlabel('time [s]');
y6 = ylabel('position [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration, posNED(:,3)- truthNED(:, 3), '.r')

l6 = legend('Truth', 'INS', 'Location', 'SouthEast');
t6 = title('Down Position Residuals');
x6 = xlabel('time [s]');
y6 = ylabel('position error [m]');
ax6 = gca;
% ax6.YLim = [-50 50];

grid
set(t6, 'FontSize', fontTitle);
set(x6, 'FontSize', fontLabel);
set(y6, 'FontSize', fontLabel);
set(l6, 'FontSize', fontLegend);

% Geoplot

figure
geoplot(data.truth.LLA(:,1), data.truth.LLA(:,2),'--')
hold on
geoplot(rad2deg(insPositionLLALog(:,1)), rad2deg(insPositionLLALog(:,2)))
