function [course, nedVelocityNorm, gnssVelocity] = gnssCourseAlignment(llaNew, llaOld, dt, nedOriginLLA)

    nedOriginLLA = [deg2rad(nedOriginLLA(1)) deg2rad(nedOriginLLA(2)) llaNew(3)];

    ecefNew = llh2ecef([deg2rad(llaNew(1)) deg2rad(llaNew(2)) llaNew(3)]);
    nedNew = ecef2ned(ecefNew, nedOriginLLA);

    ecefOld = llh2ecef([deg2rad(llaOld(1)) deg2rad(llaOld(2)) llaOld(3)]);
    nedOld = ecef2ned(ecefOld, nedOriginLLA);

    nVelocity = (nedNew(1) - nedOld(1))/dt;
    eVelocity = (nedNew(2) - nedOld(2))/dt;
    dVelocity = (nedNew(3) - nedOld(3))/dt;

    nedVelocityNorm = vecnorm([nVelocity eVelocity dVelocity]);
    course = atan2(eVelocity/nedVelocityNorm, nVelocity/nedVelocityNorm);
    gnssVelocity = [nVelocity; eVelocity; dVelocity];
    

end