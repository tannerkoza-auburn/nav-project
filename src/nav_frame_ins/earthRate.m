function omegaEI_N = earthRate(latitude)

    wEI = 7.292115e-5; % Earth angular rate (rad/s)

    omegaEI_N = wEI * [0 sin(latitude) 0; ...
                       -sin(latitude) 0 -cos(latitude); ...
                       0 cos(latitude) 0];

end