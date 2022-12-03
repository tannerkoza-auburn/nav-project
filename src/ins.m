%% INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log2";
gpsSamplingFreq = 200; % downsampled gps frequency
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
Cbn = [0.6428    0.7660         0; ...
   -0.7660    0.6428         0; ...
         0         0    1.0000]'; % unaligned with nav-frame

% Velocity
insVelocityNED = [0; 0; 0];
insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = data.gps.LLA(1, :); % initial position
mapOriginLLA = [42.294319 -83.223275 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
insPositionLLALog(1, :) = insPositionLLA; % assign initial position in log
gpsPositionLLA = data.gps.LLA(1:gpsSampleInterval:end, :);

%% INS MECHANIZATION

fprintf('nav-project: running navigation filter...\n')

for i = 2:numSamplesIMU        
    
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1); % time step (s)

    wBI_B = data.imu.angular_velocity(i, :)';
    fBI_B = data.imu.linear_acceleration(i, :)';

    gravityNED = earthGravity(insPositionLLA);

    omegaEI_N = earthRate(insPositionLLA(1));
    omegaNE_N = transportRate(insPositionLLA, insVelocityNED);

    [Cbn, insEulerAngles] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, 'dcm', 'lofi');

    fBI_N = Cbn * fBI_B;

    insVelocityNED = velocityUpdate(fBI_B, insVelocityNED, omegaEI_N, omegaNE_N, gravityNED, dt);

    insPositionLLA = gpsPositionLLA(i, :);
%     insPositionLLA = positionUpdate(insPositionLLA, insVelocityNED, dt);

    % Logging
    insPositionLLALog(i, :) = insPositionLLA;
    insEulerAnglesLog(i, :) = insEulerAngles;
    insVelocityNEDLog(i, :) = insVelocityNED';

end

%% ADDITIONAL DATA

insPositionECEFLog = llh2ecef([deg2rad(insPositionLLALog(:,1)) deg2rad(insPositionLLALog(:,2)) insPositionLLALog(:,3)]);
insPositionNEDLog = ecef2ned(insPositionECEFLog, mapOriginLLA);

%%

figure


