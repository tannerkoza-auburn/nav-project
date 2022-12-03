%% GNSS/INS Integration - FORDAV Data

clear
clc
close all

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log2";

%% LOAD DATA

fprintf('nav-project: loading and calculating additional data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% FILTER INITIALIZATION

% Time
numEpochs = length(data.truth.timeDuration);

% Alignment
originLLA = [deg2rad(42.294319) deg2rad(-83.223275) 0];
% Cbn = [0.6428    0.7660         0; ...
%    -0.7660    0.6428         0; ...
%          0         0    1.0000]'; % unaligned with nav-frame

% Velocity
initialVelocityN = [0; 0; 0];
velocityN = initialVelocityN;

% Position
mechanizedLLA = data.gps.LLA(1,:);


% Filter Status
aligned = 0;
courseCtr = 1;

% Preallocation
eulerAngles = zeros(numEpochs,3);

%% MECHANIZATION

fprintf('nav-project: running navigation filter...\n')

for i = 2:numEpochs

    % Data Extraction
    dt = data.imu.timeDuration(i) - data.imu.timeDuration(i-1);
    previousLLA = data.gps.LLA(i-1,:);
    currentLLA = data.gps.LLA(i,:);

    % Alignment
    if ~aligned

        [gnssCourse, velocityThreshold] = gnssCourseAlignment(currentLLA, previousLLA, dt, originLLA);

        if velocityThreshold > 5 && courseCtr < 100
            
            accruedCourse(courseCtr) = gnssCourse;
            courseCtr = courseCtr + 1;
        end

        if courseCtr == 99
            gnssCourse = mean(rmoutliers(accruedCourse));
            Cbn = navtools.genDCM(gnssCourse, 'z','rads')';
            aligned = 1;
        end
        
    % Mechanization
    elseif aligned

        wBI_B = data.imu.angular_velocity(i,:);
        fBI_B = data.imu.linear_acceleration(i,:)';
        gravityN = earthGravity(mechanizedLLA);
    
        omegaEI_N = earthRate(mechanizedLLA(1));
        omegaNE_N = transportRate(mechanizedLLA, velocityN);
    
        [Cbn, eulerAngles(i,:)] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, ...
            omegaNE_N, dt, 'dcm', 'lofi');
    
        fBI_N = Cbn*fBI_B;
    
        velocityN = velocityUpdate(fBI_B,velocityN,omegaEI_N,omegaNE_N,gravityN,dt);

        mechanizedLLA = positionUpdate(mechanizedLLA, velocityN, dt);

        mechanizedLLALog(i,:) = mechanizedLLA;
    end

end

data.results.timeDuration = data.truth.timeDuration;
data.results.euler = eulerAngles;

%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')

plotResults(data)
