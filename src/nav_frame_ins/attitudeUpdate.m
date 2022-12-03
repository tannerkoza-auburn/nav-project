function [Cbn, eulerAngles] = attitudeUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, type, varargin)

    if size(wBI_B, 1) ~= 3
        wBI_B = wBI_B';
    end

    wEI_N = skewInv(omegaEI_N);
    wNE_E = skewInv(omegaNE_N);
    wBI_B = wBI_B - Cbn' * (wEI_N + wNE_E); % corrected body angular rates

    switch lower(type)

        case 'dcm'

            if isempty(varargin)

                error(["Need to specify type of DCM Update: " ...
                    "'lofi' or 'precise'."])

            elseif strcmpi(varargin{1}, 'precise')

                Cbn = dcmUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, 'precise');

            elseif strcmpi(varargin{1}, 'lofi')

                Cbn = dcmUpdate(wBI_B, Cbn, omegaEI_N, omegaNE_N, dt, 'lofi');

            end

            eulerAngles = rad2deg(dcm2euler(Cbn));

        case 'quaternion' % TODO: implement quaternion update

    end

end