function velocityN = velocityUpdate(fBI_N, velocityOld, omegaEI_N, omegaNE_N, gravityN, dt)

    coriolis = (omegaNE_N + 2 * omegaEI_N); % coriolis

    fBI_N = fBI_N + gravityN - (coriolis * velocityOld); % Corrected specific force in nav-frame

    velocityN = velocityOld + (fBI_N' * dt); % Velocity update

end