function mechanizedLLA = positionUpdate(mechanizedLLA, mechanizedVelocity, dt)

latitude = mechanizedLLA(1);
longitude = mechanizedLLA(2);
altitude = mechanizedLLA(3);

velocityNorth = mechanizedVelocity(1);
velocityEast = mechanizedVelocity(2);
velocityDown = mechanizedVelocity(3);


%% Altitude

altitude  = altitude - velocityDown * dt;

if altitude < 0.0
    altitude = abs(altitude);
%     warning('pos_update: altitude is negative.')
end

%% Latitude

[R_N, ~] = earthRadius(latitude);

velocityNorth = velocityNorth / (R_N + altitude);

latitude = latitude + velocityNorth * dt;

%% Longitude

[~, R_E] = earthRadius(latitude);

velocityEast  = velocityEast / ((R_E + altitude) * cos (latitude));

longitude = longitude + velocityEast * dt;

%% Position update

mechanizedLLA = [latitude longitude altitude];

end