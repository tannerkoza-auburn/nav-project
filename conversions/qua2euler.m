function euler = qua2Euler(q)

    if size(q, 2) ~= 4
        q = q';
    end

    psi = atan2(2.*(q(:,1) .* q(:,4) + q(:,2) .* q(:,3)), (q(:,1).^2 + q(:,2).^2 - q(:,3).^2 - q(:,4).^2));
    theta = asin(2.*(q(:,1) .* q(:,3) - q(:,2) .*q(:,4)));
    phi = atan2(2.*(q(:,1) .* q(:,2) + q(:,3) .* q(:,4)), (q(:,1).^2 - q(:,2).^2 - q(:,3).^2 + q(:,4).^2));

    euler = rad2deg([psi theta phi]);

end
