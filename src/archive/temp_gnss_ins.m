%% GNSS/INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log3";
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
insEulerAngles = [0 0 0];
insEulerAnglesLog = zeros(numSamplesIMU, 3);
gpsCourseBuffer = zeros(gpsCourseBufferLength, 1);
bufferIdx = 1;

% Velocity
insVelocityNED = [0; 0; 0];
insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = [deg2rad(data.gps.LLA(1,1)) deg2rad(data.gps.LLA(1,2)) ...
    data.gps.LLA(1,3)]; % initial position
mapOriginLLA = [42.294319 -83.223275 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
insPositionLLALog(1, :) = insPositionLLA; % assign initial position in log
gpsPositionLLA = data.gps.LLA(1:gpsSampleInterval:end, :);

% Status Booleans
filterAligned = 0;

%% INS MECHANIZATION

fprintf('nav-project: running navigation filter...\n')

for i = 2:numSamplesIMU

    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1); % time step (s)

    % check for GPS measurement between IMU samples
    gpsIdx = find(gpsTime <= insTime(i) & gpsTime > insTime(i-1));

    % Alignment
    % check if filter is aligned and two gps positions are available
    if ~filterAligned && ~isempty(gpsIdx)

        [gpsCurrentCourse, gpsVelNorm, insVelocityNED] = gnssCourseAlignment(gpsPositionLLA(gpsIdx, :), gpsPositionLLA(gpsIdx-1, :), dt, mapOriginLLA);
        insVelocityNED = [0; 0; 0];
        % check if norm indicates platform is moving and if buffer isn't full
        if gpsVelNorm > gpsVelNormThreshold && bufferIdx <= gpsCourseBufferLength
            gpsCourseBuffer(bufferIdx) = gpsCurrentCourse;
            bufferIdx = bufferIdx + 1;

            % check if buffer is full
        elseif all(gpsCourseBuffer)
            meanGPSCourse = mean(rmoutliers(gpsCourseBuffer));
            Cbn = navtools.genDCM(meanGPSCourse, 'z', 'rads')'; % note transposition
            alignmentIdx = i;
            insPositionLLA = [deg2rad(gpsPositionLLA(gpsIdx, 1)) deg2rad(gpsPositionLLA(gpsIdx, 2)) gpsPositionLLA(gpsIdx, 3)]; % initalize ins position
            filterAligned = 1;
            fprintf('nav-project: filter is aligned...\n')
        end

        % Mechanize & Filter
        % check if filter is aligned
    elseif filterAligned

        % data extraction
        wBI_B = data.imu.angular_velocity(i, :);
        fBI_B = data.imu.linear_acceleration(i, :)';
        gravityNED = earthGravity(insPositionLLA);

        omegaEI_N = earthRate(insPositionLLA(1));
        omegaNE_N = transportRate(insPositionLLA, insVelocityNED);

        [Cbn, insEulerAngles] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, 'dcm', 'lofi');

        fBI_N = Cbn * fBI_B;

        insVelocityNED = velocityUpdate(fBI_N, insVelocityNED, omegaEI_N, omegaNE_N, gravityNED, dt);

        insPositionLLA = positionUpdate(insPositionLLA, insVelocityNED, dt);

    end

    % Logging
    insPositionLLALog(i, :) = insPositionLLA;
    insEulerAnglesLog(i, :) = insEulerAngles;
    insVelocityNEDLog(i, :) = insVelocityNED';

end

%% PACKAGE RESULTS DATA

data.results.timeDuration = data.imu.timeDuration;
data.results.ins.euler = insEulerAnglesLog;
data.results.ins.LLA = insPositionLLALog;
data.results.ins.velocityNED = insVelocityNEDLog;

%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')

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

plotResults(data)

% Geoplot

figure
geoplot(data.truth.LLA(:,1), data.truth.LLA(:,2),'--')
hold on
geoplot(rad2deg(insPositionLLALog(:,1)), rad2deg(insPositionLLALog(:,2)))
