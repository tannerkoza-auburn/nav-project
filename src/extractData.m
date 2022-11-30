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

%% CALCULATE NEW VALUES

out.truth.ECEF = ned2ecef(out.truth.NED, initLLA);
truthLLA = ecef2llh(out.truth.ECEF); % radians
out.truth.LLA = [rad2deg(truthLLA(:,1)) rad2deg(truthLLA(:,2)) ...
    truthLLA(:,3)];

out.gps.ECEF = llh2ecef(gpsLLA);
out.gps.NED = ecef2ned(out.gps.ECEF, initLLA);

%% REASSIGN UNMODIFIED VALUES

out.truth.time = data.pose_ground_truth.Time;
out.gps.time = data.gps.Time;
out.imu.time = data.imu.Time;
out.raw_velocity.time = data.velocity_raw.Time;

out.truth.orientation_wxyz = [data.pose_ground_truth.pose_orientation_w...
    data.pose_ground_truth.pose_orientation_x...
    data.pose_ground_truth.pose_orientation_y...
    data.pose_ground_truth.pose_orientation_z];

out.imu.linear_acceleration = [data.imu.linear_acceleration_x...
    data.imu.linear_acceleration_y data.imu.linear_acceleration_z];
out.imu.angular_velocity = [data.imu.angular_velocity_x...
    data.imu.angular_velocity_y data.imu.angular_velocity_z];
out.imu.orientation_wxyz = [data.imu.orientation_w...
    data.imu.orientation_x data.imu.orientation_y data.imu.orientation_z];

out.raw_velocity.NED = [data.velocity_raw.vector_x data.velocity_raw.vector_y ...
    data.velocity_raw.vector_z];


end