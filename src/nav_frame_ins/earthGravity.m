function gravityNED = earthGravity(lla)

    latitude = lla(1);
    altitude = lla(3);

    M = max(size(latitude));

    gravityNED = zeros(M, 3);

    %   RM, Nx1 meridian radius of curvature (North-South)(m).
    %   RN, Nx1 normal radius of curvature (East-West) (m).

    % Parameters
    R_0 = 6378137; % WGS84 Equatorial radius in meters
    R_P = 6356752.31425; % WGS84 Polar radius in meters
    e = 0.0818191908425; % WGS84 eccentricity
    f = 1 / 298.257223563; % WGS84 flattening
    mu = 3.986004418E14; % WGS84 Earth gravitational constant (m^3 s^-2)
    omega_ie_n = 7.292115E-5; % Earth rotation rate (rad/s)

    % Calculate surface gravity using the Somigliana model, (2.134)
    sinl2 = sin(latitude).^2;
    g_0 = 9.7803253359 * (1 + 0.001931853 .* sinl2) ./ sqrt(1-e^2.*sinl2);

    % Calculate north gravity using (2.140)
    gravityNED(:, 1) = -8.08E-9 .* altitude .* sin(2.*latitude);

    % East gravity is zero
    gravityNED(:, 2) = 0;

    % Calculate down gravity using (2.139)
    gravityNED(:, 3) = g_0 .* (1 - (2 ./ R_0) .* (1 + f .* (1 - 2 .* sinl2) + ...
        (omega_ie_n^2 .* R_0^2 .* R_P ./ mu)) .* altitude + (3 .* altitude.^2 ./ R_0^2));

end