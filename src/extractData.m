function out = extractData(filePath)

%% LOAD DATA

data = load(filePath);

%% EXTRACT & ASSIGN NECESSARY VALUES

initLLA = [deg2rad(42.294319) deg2rad(-83.223275) 0]; % radians (hardcoded map origin)

truthN = data.pose_ground_truth.pose_position_x;
truthE = data.pose_ground_truth.pose_position_y;
truthD = data.pose_ground_truth.pose_position_z;
out.truth.NED = [truthN truthE truthD];

gpsLat = data.gps.latitude;
gpsLon = data.gps.longitude;
gpsAlt = data.gps.altitude;
gpsLLA = [deg2rad(gpsLat) deg2rad(gpsLon) gpsAlt]; % radians
out.gps.LLA = [gpsLat gpsLon gpsAlt];

out.truth.quaternion = [data.pose_ground_truth.pose_orientation_w...
    data.pose_ground_truth.pose_orientation_x...
    data.pose_ground_truth.pose_orientation_y...
    data.pose_ground_truth.pose_orientation_z];

%% CALCULATE NEW VALUES

out.truth.ECEF = ned2ecef(out.truth.NED, initLLA);
truthLLA = ecef2llh(out.truth.ECEF); % radians
out.truth.LLA = [rad2deg(truthLLA(:,1)) rad2deg(truthLLA(:,2)) ...
    truthLLA(:,3)];

out.gps.ECEF = llh2ecef(gpsLLA);
out.gps.NED = ecef2ned(out.gps.ECEF, initLLA);

out.truth.euler = qua2Euler(out.truth.quaternion);

%% REASSIGN UNMODIFIED VALUES

out.truth.timeEpoch = data.pose_ground_truth.Time;
out.truth.timeDuration = data.pose_ground_truth.Time ...
    - data.pose_ground_truth.Time(1);
out.gps.timeEpoch = data.gps.Time;
out.gps.timeDuration = data.gps.Time ...
    - data.gps.Time(1);
out.imu.timeEpoch = data.imu.Time;
out.imu.timeDuration = data.imu.Time ...
    - data.imu.Time(1);
out.raw_velocity.timeEpoch = data.velocity_raw.Time;
out.raw_velocity.timeDuration = data.velocity_raw.Time ...
    - data.velocity_raw.Time(1);

out.imu.linear_acceleration = [data.imu.linear_acceleration_x...
    data.imu.linear_acceleration_y data.imu.linear_acceleration_z];
out.imu.angular_velocity = [data.imu.angular_velocity_x...
    data.imu.angular_velocity_y data.imu.angular_velocity_z];
out.imu.orientation_wxyz = [data.imu.orientation_w...
    data.imu.orientation_x data.imu.orientation_y data.imu.orientation_z];

out.raw_velocity.NED = [data.velocity_raw.vector_x data.velocity_raw.vector_y ...
    data.velocity_raw.vector_z];


end