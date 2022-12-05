%% INS Integration - Sim NaveGo Data

clear
clc
close all

%% LOAD DATA

load imu1.mat
load gnss.mat
load ref.mat

%% INITIALIZATION

% Time
numSamplesIMU = length(imu1.t);

% Attitude
Cbn = eye(3);
q = [0; 0; 0; 1];

insEulerAnglesLog = zeros(numSamplesIMU, 3);

% Velocity
insVelocityNED = [0 0 0];

insVelocityNEDLog = zeros(numSamplesIMU, 3);

% Position
insPositionLLA = [ref.lat(1) ref.lon(1) ref.h(1)];

insPositionLLALog = zeros(numSamplesIMU, 3);

for i = 2:numSamplesIMU

    % Extract Sensor Data
    dt = imu1.t(i) - imu1.t(i-1);
    wb = imu1.wb(i,:);
    fb = imu1.fb(i, :);

    % Attitude Update
    gravityNED = gravity(insPositionLLA(1),insPositionLLA(3));
    omegaEI_N = earth_rate(insPositionLLA(1));
    omegaNE_N = transport_rate(insPositionLLA(1), insVelocityNED(1), ...
        insVelocityNED(2), insPositionLLA(3));
    [~, Cbn, insEulerAngles] = att_update(wb, Cbn, q, omegaEI_N, ...
        omegaNE_N, dt, 'dcm');

    % Velocity Update
    fBI_N = Cbn* fBI_B;
    insVelocityNED = vel_update(fb, insVelocityNED, omegaEI_N, ...
        omegaNE_N, gravityNED, dt);

    % Position Update
    insPositionLLA = pos_update(insPositionLLA, insVelocityNED, dt);

end