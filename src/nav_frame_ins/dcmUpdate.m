function Cbn = dcmUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, type)

    switch lower(type)

        case 'lofi'

            omegaBI_B = skew(wBI_B);

            Cbn = Cbn * (eye(3) + omegaBI_B * dt) - (omegaEI_N + omegaNE_N) * Cbn * dt;

        case'precise'

            deltaTheta = wBI_B * dt; % attitude increment (rad)
            magDeltaTheta = norm(deltaTheta);
            skewDeltaTheta = skew(deltaTheta);

            if magDeltaTheta < 1e-8

                Cbb = eye(3);

            else

                Cbb = expm(skewDeltaTheta);

            end

            Cbn = Cbn * Cbb;

            % Brute-Force Orthogonalization, Groves, Eq 5.79
            c1 = Cbn(:, 1);
            c2 = Cbn(:, 2);
            c3 = Cbn(:, 3);

            c1 = c1 - 0.5 * (c1' * c2) * c2 - 0.5 * (c1' * c3) * c3;
            c2 = c2 - 0.5 * (c1' * c2) * c1 - 0.5 * (c2' * c3) * c3;
            c3 = c3 - 0.5 * (c1' * c3) * c1 - 0.5 * (c2' * c3) * c2;

            % Brute-Force Normalization, Groves, Eq 5.80
            c1 = c1 / sqrt(c1'*c1);
            c2 = c2 / sqrt(c2'*c2);
            c3 = c3 / sqrt(c3'*c3);

            Cbn = [c1, c2, c3];

    end

end