function dataset = make_dataset(num_samples, sigma)
    start_time = datetime(2024, 8, 29, 0, 0, 0);
    stop_time = start_time + seconds(num_samples);
    
    % Create the satellite scenario
    sc = satelliteScenario(start_time, stop_time, num_samples, 'AutoSimulate', false);

    [sv1, sv2] = get_virtual_satellite(sc);
    gps_sv = get_gps_satellite(sc)';

    % v = satelliteScenarioViewer(sc,'ShowDetails',true);
    % play(sc);

    times = start_time + seconds((0:num_samples-1));
    true_positions_sat1 = zeros(num_samples, 3);
    true_positions_sat2 = zeros(num_samples, 3);

    target_prn = 1:31;

    measurements = zeros(length(target_prn), 2, num_samples);
    position_sv= zeros(length(target_prn), 3, num_samples);
    for i = 1:num_samples
        % pos1 = states(sv1, times(i), 'CoordinateFrame', 'ecef');
        % pos2 = states(sv2, times(i), 'CoordinateFrame', 'ecef');
        pos1 = [-3.119992580788137e+06, 4.086868171897103e+06, 3.761594895585738e+06]';
        pos2 = [-3.000992580788137e+06, 4.000868171897103e+06, 3.000594895585738e+06]';
        idx = 1;

        pos_gps = states(gps_sv, times(i), "CoordinateFrame", 'ecef');
        for j = 1:31
            if ismember(j, target_prn)
                rotated_pos_gps = rotate_gps(pos_gps(:, 1, j))

                measurements(idx, 1, i) = generate_pr(rotated_pos_gps, pos1, sigma);
                measurements(idx, 2, i) = generate_pr(rotated_pos_gps, pos2, sigma);

                position_sv(idx, :, i) = pos_gps(:,:,j);
                idx = idx + 1;
            end
        end

        true_positions_sat1(i, :) = pos1(1:3); % X, Y, Z coordinates
        true_positions_sat2(i, :) = pos2(1:3); % X, Y, Z coordinates
    end

    dataset.sat1_positions = true_positions_sat1;
    dataset.sat2_positions = true_positions_sat2;
    dataset.measurements = measurements;
    dataset.gps_positions = position_sv;
    dataset.times = times;
end


function rotated_cord = rotate_gps(gps_pos)
    rotated_cord = gps_pos;

    c = 299792458; % Speed of light in m/s
    omega = 7.292115E-5;  % Earth rotation rate in rad/s
    
    approx_range = 1;

    C = [1, omega * approx_range / c, 0;...
         -omega * approx_range / c, 1, 0;...
         0, 0, 1];

    inv_C = inv(C);
end


function pr = generate_pr(gps_pos, sat_pos, sigma)
    distance = norm(gps_pos - sat_pos);
    
    % Generate Gaussian noise with mean 0 and standard deviation sigma
    noise = sigma * randn;
    
    % Calculate the pseudorange
    pr = distance + noise;
end



function [sv1, sv2] = get_virtual_satellite(scenario)
    sv1 = satellite(scenario, './data/tle1.txt', ...
                    "OrbitPropagator","sgp4");
    sv2 = satellite(scenario, './data/tle2.txt', ...
                    "OrbitPropagator","sgp4");

end

function sv = get_gps_satellite(sceneario)
    sv = satellite(sceneario, './data/gpsalmanac.txt')

end