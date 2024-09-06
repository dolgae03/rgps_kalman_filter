function tdcp_vec = calculate_tdcp(prev_pos, vel, dt, prev_carrier, prev_gps_pos, curr_carrier, curr_gps_pos)

    c = 299792458;           % Speed of light in m/s
    omega = 7.292115E-5;     % Earth rotation rate in rad/s
    
    num_sv = size(prev_carrier, 1); % Number of satellites
    tdcp_vec = zeros(num_sv, 1);    % Initialize TDCP vector for all satellites
    
    % Update receiver's position based on velocity and time difference
    curr_pos = prev_pos + vel * dt;   % Current position propagated by velocity and time

    for k = 1:num_sv
        % Get the satellite positions at previous and current epochs
        sv_pos_prev = prev_gps_pos(:, k);
        sv_pos_curr = curr_gps_pos(:, k);
        
        % Apply Earth rotation correction matrix based on the previous satellite position
        approx_r = sv_pos_prev - prev_pos; % Approximate range between receiver and satellite at the previous epoch
        approx_range = sqrt(approx_r' * approx_r);
        
        % Earth rotation correction matrix
        C = [1, omega * approx_range / c, 0; ...
             -omega * approx_range / c, 1, 0; ...
             0, 0, 1];
         
        % Corrected satellite positions due to Earth's rotation
        r_prev = C * sv_pos_prev - prev_pos;
        r_curr = C * sv_pos_curr - curr_pos; % Use the updated current position
        
        % Calculate the range between receiver and satellite at both epochs
        range_prev = sqrt(sum((r_prev).^2));
        range_curr = sqrt(sum((r_curr).^2));
        
        % Line-of-sight vectors for both epochs
        los_prev = r_prev / range_prev;
        los_curr = r_curr / range_curr;
        
        % Compute motion difference (satellite motion)
        sat_motion = (los_curr' * sv_pos_curr) - (los_prev' * sv_pos_prev);
        g_motion = (los_curr' * curr_pos) - (los_prev' * curr_pos);
        d_motion = sat_motion - g_motion;
        
        % Calculate TDCP: (current - previous carrier phase) - satellite motion
        tdcp_measured = (curr_carrier(k) - prev_carrier(k)) - d_motion;
        
        % Store the result for each satellite
        tdcp_vec(k) = tdcp_measured;
    end
end
