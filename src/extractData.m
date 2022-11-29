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

end