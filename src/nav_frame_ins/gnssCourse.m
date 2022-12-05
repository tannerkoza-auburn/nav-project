function [course, nedVelocityNorm, gnssVelocity] = gnssCourse(llaNew, llaOld, dt, nedOriginLLA)

    nedOriginLLA = [deg2rad(nedOriginLLA(1)) deg2rad(nedOriginLLA(2)) 0];

    ecefNew = llh2ecef(llaNew);
    nedNew = ecef2ned(ecefNew, nedOriginLLA);

    ecefOld = llh2ecef(llaOld);
    nedOld = ecef2ned(ecefOld, nedOriginLLA);

    nVelocity = (nedNew(1) - nedOld(1))/dt;
    eVelocity = (nedNew(2) - nedOld(2))/dt;
    dVelocity = (nedNew(3) - nedOld(3))/dt;

    nedVelocityNorm = vecnorm([nVelocity eVelocity dVelocity]);
    course = atan2(eVelocity/nedVelocityNorm, nVelocity/nedVelocityNorm);
    gnssVelocity = [nVelocity; eVelocity; dVelocity];
    

end