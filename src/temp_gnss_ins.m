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
insPositionLLA = data.gps.LLA(1, :); % initial position
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

        % check if norm indicates platform is moving and if buffer isn't full
        if gpsVelNorm > gpsVelNormThreshold && bufferIdx <= gpsCourseBufferLength
            gpsCourseBuffer(bufferIdx) = gpsCurrentCourse;
            bufferIdx = bufferIdx + 1;

            % check if buffer is full
        elseif all(gpsCourseBuffer)
            meanGPSCourse = mean(rmoutliers(gpsCourseBuffer));
            Cbn = navtools.genDCM(meanGPSCourse, 'z', 'rads')'; % note transposition
            alignmentIdx = i;
            insPositionLLA = gpsPositionLLA(gpsIdx, :); % initalize ins position
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

        insVelocityNED = velocityUpdate(fBI_B, insVelocityNED, omegaEI_N, omegaNE_N, gravityNED, dt);

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

plotResults(data)
