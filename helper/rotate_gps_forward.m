function rotated_cord = rotate_gps_forward(gps_pos, ref_pos)
    c = 299792458; % Speed of light in m/s
    omega = 7.292115E-5;  % Earth rotation rate in rad/s
    
    approx_range = norm(gps_pos - ref_pos);

    C_forward = [1, omega * approx_range / c, 0;...
         -omega * approx_range / c, 1, 0;...
         0, 0, 1];

    rotated_cord = C_forward * gps_pos;
end
