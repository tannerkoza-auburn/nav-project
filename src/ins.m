%% INS Integration - FORDAV Data

clear
clc
close all 

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log3";

%% LOAD DATA

fprintf('nav-project: loading and calculating data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')


%% INITIALIZATION

numEpochs = length(data.imu.timeDuration);
eulerLog = zeros(numEpochs, 3);
velLog = zeros(numEpochs, 3);
posLog = zeros(numEpochs, 3);

%% NAV MECHANIZATION OF IMU

%initialize
v_eb_n=[0 0 0];
r_eb_n=[0 0 0];
C_b_n=  [0.6428   -0.7660         0;
    0.7660    0.6428         0;
         0         0    1.0000];
Lb=deg2rad(data.truth.LLA(1,1));
lamb=deg2rad(data.truth.LLA(1,2));
hb=data.truth.LLA(1,3);

nedPos = zeros(3,1);

for t=2:length(data.imu.timeEpoch)

    dt=data.imu.timeEpoch(t)-data.imu.timeEpoch(t-1);

%update radii
[Rn, Re] = earthRadius(deg2rad(Lb));

%angular velocities
omega_ib_b=data.imu.angular_velocity(t,:);
omegak_ib_b=[0 -omega_ib_b(3) omega_ib_b(2);... 
     omega_ib_b(3) 0 -omega_ib_b(1);...
     -omega_ib_b(2) omega_ib_b(1) 0];

omega_ie_e=7.292115e-5; %rad/s
omegak_ie_n=omega_ie_e*[0 sin(Lb) 0; -sin(Lb) 0 -cos(Lb); 0 cos(Lb) 0];
omega_en_n=[v_eb_n(2)/(Re*Lb+hb) -v_eb_n(1)/(Re*Lb+hb)...
    (-v_eb_n(2)*tan(Lb))/(Re*Lb+hb)];
omegak_en_n=[0 -omega_en_n(3) omega_en_n(2);... 
    omega_en_n(3) 0 -omega_en_n(1);...
    -omega_en_n(2) omega_en_n(1) 0];

%attitude
C_b_n=C_b_n*(eye(3)+omegak_ib_b*dt)-(omegak_ie_n+omegak_en_n)*C_b_n*dt;
euler = dcm2euler(C_b_n);

 %specific force
f_ib_b=data.imu.linear_acceleration(t,:);
f_ib_n=C_b_n*f_ib_b';

%acceleration
    acc_eb_n=f_ib_n-(omegak_en_n+2*omegak_ie_n)*v_eb_n';
%velocity
    v_eb_n=(v_eb_n'+acc_eb_n*dt)';


    hb=hb-dt*v_eb_n(3);

    if hb < 0.0
    hb = abs(hb);
    end

    Lb=Lb+dt*(v_eb_n(1)/(Rn+hb));
lamb=lamb+dt*(v_eb_n(2)/ (cos(Lb)*(Re+hb)));

% Logging
nedPos = nedPos + v_eb_n'*dt;
nedLog(t,:) = nedPos;
eulerLog(t,:) = euler;
velLog(t,:) = v_eb_n';
posLog(t,:) = [Lb lamb hb];
end


 geoplot(posLog(2:end,1), posLog(2:end,2))
hold on
geoplot(data.truth.LLA(:,1),data.truth.LLA(:,2))
figure
plot(velLog)
figure
plot(eulerLog)
 %% ECEF MECHANIZATION OF IMU
% 
% %initialize
% v_eb_e(2,:)=[0 0 0];
% r_eb_e(2,:)=[0 0 0];
% C_b_e=eye(3);
% 
% for t=2:length(data.imu.time)
% 
%     dt=data.imu.time(t)-data.imu.time(t-1);
% 
% %angular velocity
% omega_ie_e=7.292115e-5; %rad/s
% omega_ib_b=data.imu.angular_velocity(t,:);
% omegak_ie_e=[0 -omega_ie_e 0; omega_ie_e 0 0; 0 0 0];
% omegak_ib_b=[0 -omega_ib_b(3) omega_ib_b(2);... 
%     omega_ib_b(3) 0 -omega_ib_b(1);...
%     -omega_ib_b(2) omega_ib_b(1) 0];
% 
% %attitude
% C_b_e=C_b_e*(eye(3)+omegak_ib_b*dt)-omegak_ie_e*C_b_e*dt;
% 
%  %specific force
% f_ib_b=data.imu.linear_acceleration(t,:);
% f_ib_e=C_b_e*f_ib_b';
% 
% %acceleration
%     acc_eb_e(t,:)=(-2*omegak_ie_e*v_eb_e(t-1,:)'+f_ib_e)';
% %velocity
%     v_eb_e(t,:)=v_eb_e(t-1,:)'+acc_eb_e(t-1,:)'*dt;
% %position
%     r_eb_e(t,:)=r_eb_e(t-1,:)'+v_eb_e(t-1,:)'*dt+acc_eb_e(t-1)'*(dt^2/2);
% end
