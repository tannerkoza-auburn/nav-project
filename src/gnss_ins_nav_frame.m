%% GNSS/INS Integration - FORDAV Data

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
Cbn = [0.6428 0.7660 0; ...
    -0.7660 0.6428 0; ...
    0 0 1.0000]'; % unaligned with nav-frame

% Cbn = [  0.6887   -0.7251         0;
%     0.7251    0.6887         0;
%          0         0    1.0000];

insEulerAnglesLog = zeros(numSamplesIMU, 3);

% Velocity
insVelocityNED = [0; 0; 0];

insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = data.gps.LLA(1, :); % initial position
gpsPositionLLA = data.gps.LLA(1:gpsSampleInterval:end, :);
mapOriginLLA = [42.294319 -83.223275 0];

insPositionLLALog = zeros(numSamplesIMU, 3);
insPositionLLALog(1, :) = insPositionLLA;

% Status Booleans
insAligned = 0;
gpsMeasStatus = 0;

%% INS MECHANIZATION

for i = 2:numSamplesIMU

    % Extract Sensor Data
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    wBI_B = data.imu.angular_velocity(i, :)';
    fBI_B = data.imu.linear_acceleration(i, :)';

    % Calculate Angular Rates to Remove
    % NOTE: gravity is already removed from the specific force measurements
    omegaEI_N = earthRate(insPositionLLA(1));
    omegaNE_N = transportRate(insPositionLLA, insVelocityNED);

    % Attitude Update
    [Cbn, insEulerAngles] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, 'dcm', 'precise');

    % Velocity Update
    fBI_N = Cbn * fBI_B;
    insVelocityNED = velocityUpdate(fBI_N, insVelocityNED, omegaEI_N, omegaNE_N, dt);

    % Position Update
    insPositionLLA = positionUpdate(insPositionLLA, insVelocityNED, dt);

    %% KALMAN FILTER UPDATE

    gpsIdx = find(gpsTime <= insTime(i) & gpsTime > insTime(i-1));
    gpsMeasStatus = ~isempty(gpsIdx);

    if gpsMeasStatus && gpsIdx > 1

        [R_N, R_E] = earthRadius(insPositionLLA(1));

        Tpr = diag([(R_N + insPositionLLA(3)), (R_E + insPositionLLA(3)) * cos(insPositionLLA(1)), -1]);

        zp = Tpr * (insPositionLLA - gpsPositionLLA(gpsIdx,:);

        zv = (vel_e(i,:) - gnss.vel(gdx,:) - ((omega_ie_n + omega_en_n) * (DCMbn * gnss.larm ))' ...
            + (DCMbn * skewm(wb_corrected) * gnss.larm )' )';
    end

    % Logging
    insPositionLLALog(i, :) = insPositionLLA;
    insEulerAnglesLog(i, :) = insEulerAngles;
    insVelocityNEDLog(i, :) = insVelocityNED';

end