function velocityN = velocityUpdate(fBI_N, velocityOld, omegaEI_N, omegaNE_N, dt)

    %NOTE: the acceleration due to gravity is not an input because it is
    %removed from FORDAV sensor data
    
    coriolis = (omegaNE_N + 2 * omegaEI_N); % coriolis

    fBI_N = fBI_N  - (coriolis * velocityOld); % Corrected specific force in nav-frame

    velocityN = velocityOld + (fBI_N * dt); % Velocity update

end