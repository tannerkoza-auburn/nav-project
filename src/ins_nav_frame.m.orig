%% INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log2";
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
mapOriginLLA = [42.294319 -83.223275 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
insPositionLLALog(1,:) = insPositionLLA;

% Status Booleans
insAligned = 0;
gpsMeasStatus = 0;

for i = 2:numSamplesIMU

    % Extract Sensor Data
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    wBI_B = data.imu.angular_velocity(i, :)';
    fBI_B = data.imu.linear_acceleration(i, :)';

%     % Dynamic Alignment
%     % check if filter is aligned and two gps positions are available
%     gpsIdx = find(gpsTime <= insTime(i) & gpsTime > insTime(i-1));
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

%% ERROR CALCULATIONS

truthEulerInterp = interp1(data.truth.timeEpoch, data.truth.euler, data.imu.timeEpoch);
% [truthEulerInterp, eulerIdx] = remoutliers(truthEulerInterp)


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
figure("Name", "Yaw")
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 1), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 3), '.')
hold on

l3 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3 = title(' Yaw');
x3 = xlabel('time [s]');
y3 = ylabel('yaw [deg]');
ax3 = gca;

grid
set(t3, 'FontSize', fontTitle);
set(x3, 'FontSize', fontLabel);
set(y3, 'FontSize', fontLabel);
set(l3, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration(2:end), truthEulerInterp(2:end,1) - insEulerAnglesLog(2:end, 3) , '.k')

% l4 = legend('Truth', 'INS', 'Location', 'SouthEast');
t3E = title(' Yaw Error');
x3E = xlabel('time [s]');
y3E = ylabel('yaw error [deg]');
a3E = gca;

grid
a3E.YLim = [-5 5];
set(t3E, 'FontSize', fontTitle);
set(x3E, 'FontSize', fontLabel);
set(y3E, 'FontSize', fontLabel);
% set(l4, 'FontSize', fontLegend);

figure('Name','Pitch')
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 2), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 2), '.')
hold on

l4 = legend('Truth', 'INS', 'Location', 'SouthEast');
t4 = title(' Pitch');
x4 = xlabel('time [s]');
y4 = ylabel('pitch [deg]');
ax4 = gca;

grid
set(t4, 'FontSize', fontTitle);
set(x4, 'FontSize', fontLabel);
set(y4, 'FontSize', fontLabel);
set(l4, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration(2:end), truthEulerInterp(2:end,2) - insEulerAnglesLog(2:end, 2) , '.k')

% l4 = legend('Truth', 'INS', 'Location', 'SouthEast');
t4E = title(' Pitch Error');
x4E = xlabel('time [s]');
y4E = ylabel('pitch error [deg]');
a4E = gca;
a4E.YLim = ax4.YLim;

grid
set(t4E, 'FontSize', fontTitle);
set(x4E, 'FontSize', fontLabel);
set(y4E, 'FontSize', fontLabel);
% set(l4, 'FontSize', fontLegend);


figure('Name','Roll')
subplot(2, 1, 1)
plot(data.truth.timeDuration, data.truth.euler(:, 3), '.k')
hold on
plot(data.imu.timeDuration, insEulerAnglesLog(:, 1), '.')
hold on

l5 = legend('Truth', 'INS', 'Location', 'SouthEast');
t5 = title(' Roll');
x5 = xlabel('time [s]');
y5 = ylabel('roll [deg]');
ax5 = gca;

grid
set(t5, 'FontSize', fontTitle);
set(x5, 'FontSize', fontLabel);
set(y5, 'FontSize', fontLabel);
set(l5, 'FontSize', fontLegend);

subplot(2, 1, 2)
plot(data.imu.timeDuration(2:end), truthEulerInterp(2:end,3) - insEulerAnglesLog(2:end, 1) , '.k')

% l4 = legend('Truth', 'INS', 'Location', 'SouthEast');
t5E = title(' Roll Error');
x5E = xlabel('time [s]');
y5E = ylabel('roll error [deg]');
a5E = gca;
a5E.YLim = ax5.YLim;

grid
set(t5E, 'FontSize', fontTitle);
set(x5E, 'FontSize', fontLabel);
set(y5E, 'FontSize', fontLabel);
% set(l4, 'FontSize', fontLegend);


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

% Geoplot

figure('Name',"Geoplot Trajectory")
geoplot(data.truth.LLA(:,1), data.truth.LLA(:,2),'--k','DisplayName','Truth')
hold on
geoplot(rad2deg(insPositionLLALog(:,1)), rad2deg(insPositionLLALog(:,2)),'b','DisplayName','INS')
legend('Location','southeast')
geobasemap streets




