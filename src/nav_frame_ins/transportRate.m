function omegaNE_N = transportRate(lla, velocityNED)

    latitude = lla(1);
    altitude = abs(lla(3));

    velocityNorth = velocityNED(1); % nav-frame velocity (m/s)
    velocityEast = velocityNED(2);

    [R_N, R_E] = earthRadius(latitude); % radii of curvature (m)

    wNE_N(1) = velocityEast / (R_E * latitude + altitude); % angular rate nav-frame relative to ECEF (rad/s)
    wNE_N(2) = -velocityNorth / (R_N * latitude + altitude);
    wNE_N(3) = -velocityEast * tan(latitude) / (R_E * latitude + altitude);

    omegaNE_N = skew(wNE_N);

end