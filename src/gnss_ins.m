%% GNSS/INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V2-Log2";

%% LOAD DATA

fprintf('nav-project: loading and calculating data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% FILTER INITIALIZATION

% Time
numEpochs = length(data.truth.timeDuration);

% Alignment
Cbn = [0.6428    0.7660         0; ...
   -0.7660    0.6428         0; ...
         0         0    1.0000]'; % unaligned with nav-frame

% Velocity
initialVelocityN = [0; 0; 0];
velocityN = initialVelocityN;

% Position


% Preallocation
eulerAngles = zeros(numEpochs,3);

%% MECHANIZATION

fprintf('nav-project: running navigation filter...\n')

for i = 2:numEpochs

    % Data Extraction
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    LLA = data.gps.LLA(i,:);
    wBI_B = data.imu.angular_velocity(i,:);
    fBI_B = data.imu.linear_acceleration(i,:)';
    gravityN = earthGravity(LLA);

    omegaEI_N = earthRate(LLA(1));
    omegaNE_N = transportRate(LLA, velocityN);

    [Cbn, eulerAngles(i,:)] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, ...
        omegaNE_N, dt, 'dcm', 'lofi');

    fBI_N = Cbn*fBI_B;

    velocityN = velocityUpdate(fBI_B,velocityN,omegaEI_N,omegaNE_N,gravityN,dt);

end

data.results.timeDuration = data.truth.timeDuration;
data.results.euler = eulerAngles;

%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')

plotResults(data)
