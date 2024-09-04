function dataset = make_dataset(num_samples, sigma_pr, sigma_range)
    start_time = datetime(2024, 8, 29, 0, 0, 0);
    stop_time = start_time + seconds(num_samples);
    
    % Create the satellite scenario
    sc = satelliteScenario(start_time, stop_time, num_samples, 'AutoSimulate', false);

    [sv1, sv2] = get_virtual_satellite(sc);
    gps_sv = get_gps_satellite(sc)';

    % v = satelliteScenarioViewer(sc,'ShowDetails',true);
    % play(sc);

    times = start_time + seconds((1:num_samples));
    true_positions_sat1 = zeros(num_samples, 3);
    true_positions_sat2 = zeros(num_samples, 3);

    target_prn = 1:31;

    pr_mes = zeros(length(target_prn), 2, num_samples);
    range_mes = zeros(2, 1, num_samples);
    position_sv= zeros(length(target_prn), 3, num_samples);
    for i = 1:num_samples
        pos1 = states(sv1, times(i), 'CoordinateFrame', 'ecef');
        pos2 = states(sv2, times(i), 'CoordinateFrame', 'ecef');
        idx = 1;

        pos_gps = states(gps_sv, times(i), "CoordinateFrame", 'ecef');
        tau = norm(pos_gps(:, 1, 1) - pos1) / 299792458;
        
        pos_gps_when_send_signal = states(gps_sv, times(i) - seconds(tau), "CoordinateFrame", 'ecef');

        for j = 1:31
            if ismember(j, target_prn)
                pr_mes(idx, 1, i) = generate_pr(pos_gps_when_send_signal(:, 1, j), pos1, sigma_pr);
                pr_mes(idx, 2, i) = generate_pr(pos_gps_when_send_signal(:, 1, j), pos2, sigma_pr);

                range_mes(1, 1, i) = generate_range(pos1, pos2, sigma_range);
                range_mes(2, 1, i) = generate_range(pos1, pos2, sigma_range);

                position_sv(idx, :, i) = pos_gps(:, 1, j);
                idx = idx + 1;
            end
        end

        true_positions_sat1(i, :) = pos1(1:3); % X, Y, Z coordinates
        true_positions_sat2(i, :) = pos2(1:3); % X, Y, Z coordinates
    end

    dataset.sat1_positions = true_positions_sat1;
    dataset.sat2_positions = true_positions_sat2;
    dataset.pr_mes = pr_mes;
    dataset.range_mes = range_mes;
    dataset.gps_positions = position_sv;
    dataset.times = times;
end


function pr = generate_pr(gps_pos, sat_pos, sigma)
    distance = norm(gps_pos - sat_pos);
    
    % Generate Gaussian noise with mean 0 and standard deviation sigma
    noise = sigma * randn;
    
    % Calculate the pseudorange
    pr = distance + noise;
end

function range = generate_range(pos1, pos2, sigma)
    distance = norm(pos1 - pos2);

    noise = sigma * randn;
    
    range = distance + noise;
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