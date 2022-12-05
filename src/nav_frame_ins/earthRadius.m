function [R_N, R_E] = earthRadius(latitude)

    R_0 = 6378137.0; % equatorial radius (m)
    e = 0.0818191908425; % eccentricity

    R_N = R_0 * (1 - e^2) / (1 - e^2 * sin(latitude)^2)^(3 / 2); % meridian radius of curvature (m)
    R_E = R_0 / sqrt(1-e^2*sin(latitude)^2); % transverse radius of curvature (m)

end