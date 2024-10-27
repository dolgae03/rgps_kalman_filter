function dataset = make_dataset(num_samples, sigma_pr, sigma_range, sv_num)
    start_time = datetime(2024, 9, 8, 14, 30, 0);

    times = start_time + seconds((1:num_samples));

    pr_mes = cell(num_samples, sv_num);
    carrier_mes = cell(num_samples, sv_num);
    range_mes = cell(num_samples, sv_num);

    [gps_pos, sv_pos, sv_vel] = make_position_data(start_time, num_samples, sv_num);
    
    for i = 1:num_samples
        for k = 1:sv_num
            pos = sv_pos{i, k}(:, 1);
        
            pos_gps = gps_pos{i, k};


            for j = 1:size(pos_gps, 2)
                if ~any(isnan(pos_gps(:, j)))
                    pos_gps_when_send_signal = rotate_gps_forward(pos_gps(:, j), pos);
                    pr_mes{i, k}(j, 1) = generate_pr(pos_gps_when_send_signal, pos, sigma_pr);
                    carrier_mes{i, k}(j, 1) = generate_carrier(pos_gps_when_send_signal, pos, 0.02);
                else
                    pr_mes{i, k}(j, 1) = nan;
                    carrier_mes{i, k}(j, 1) = nan;
                end
            end
            
            if k < sv_num
                pos_next = sv_pos{i, k+1}(:, 1);
                range_mes{i, k}(1, 1) = generate_range(pos, pos_next, sigma_range);
                range_mes{i, k}(2, 1) = generate_range(pos, pos_next, sigma_range);
            end
        end
        
    end

    dataset.sat_positions = sv_pos;
    dataset.sat_velocity = sv_vel;
    dataset.pr_mes = pr_mes;
    dataset.range_mes = range_mes;
    dataset.carrier_mes = carrier_mes;
    dataset.gps_positions = gps_pos;
    dataset.times = times;
end

function [gps_pos, sv_pos, sv_vel] = make_position_data(start_time, num_samples, leo_sat_num)
    % 파일 이름 생성
    folder_name = './data/position_data';
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);  % data 폴더가 없으면 생성
    end
    % 날짜와 샘플 수 기반으로 파일 이름 생성
    file_name = sprintf('%s/position_velocity_data_%s_%d_%d.mat', folder_name, datestr(start_time, 'yyyymmdd_HHMMSS'), num_samples, leo_sat_num);
    
    % 파일이 존재하는지 확인
    if exist(file_name, 'file')
        % 파일이 존재하면 데이터를 로드
        disp('파일이 존재합니다. 데이터를 로드합니다...');
        loaded_data = load(file_name);
        gps_pos = loaded_data.gps_pos;
        sv_pos = loaded_data.sv_pos;
        sv_vel = loaded_data.sv_vel;
    else
        % 파일이 존재하지 않으면 데이터를 생성
        disp('파일이 존재하지 않습니다. 데이터를 생성합니다...');
        stop_time = start_time + seconds(num_samples);

        % 위성 시나리오 생성
        sc = satelliteScenario(start_time, stop_time, num_samples, 'AutoSimulate', false);
        
        % 가상 위성 1, 2를 얻음
        leo_sv = get_virtual_satellite(sc, leo_sat_num);

        % GPS 위성을 얻음
        gps_sv = get_gps_satellite(sc)';

        % 시뮬레이션 시간 계산
        times = start_time + seconds((1:num_samples));

        % v = satelliteScenarioViewer(sc,'ShowDetails',true);
        % play(sc);

        sv_pos = cell(num_samples, leo_sat_num);
        sv_vel = cell(num_samples, leo_sat_num);
        gps_pos = cell(num_samples, leo_sat_num);

        elevation_threshold = 30;

        for i = 1:num_samples
            % 각 시점에서 가상 위성 1과 2의 위치를 가져옴
            for k = 1:leo_sat_num
                [pos, vel] = states(leo_sv{k}, times(i), 'CoordinateFrame', 'ecef');
                sv_pos{i, k}(:, 1) = pos;
                sv_vel{i, k}(:, 1) = vel;
            end

            pos_gps = states(gps_sv, times(i), 'CoordinateFrame', 'ecef');

            % 각 GPS 위성에 대해 Elevation 각도 체크 후 위치 계산
            for j = 1:31
                for k = 1:leo_sat_num
                    if calculate_elevation(sv_pos{i, k}(:, 1), pos_gps(:, 1, j)) > elevation_threshold
                        gps_pos{i, k}(:, j) = pos_gps(:, 1, j);
                    else
                        gps_pos{i, k}(:, j) = [nan; nan; nan];
                    end
                end
            end
        end

        % 데이터를 파일로 저장
        save(file_name, 'gps_pos', 'sv_pos', 'sv_vel');
        disp(['데이터가 저장되었습니다: ' file_name]);
    end
end

function pr = generate_carrier(gps_pos, sat_pos, sigma)
    c = 299792458;           % Speed of light in m/s
    f = 1575.42e6;
    lambda =  c / f;

    exact_integer = norm(gps_pos - sat_pos) / lambda;
   
    % Generate Gaussian noise with mean 0 and standard deviation sigma
    noise = sigma * randn;
    N = 12315;
    
    % Calculate the pseudorange
    pr = exact_integer + noise / lambda;
end

function pr = generate_pr(gps_pos, sat_pos, sigma)
    distance = norm(gps_pos - sat_pos);

    clock_bias = 3;
    
    % Generate Gaussian noise with mean 0 and standard deviation sigma
    noise = sigma * randn;
    
    % Calculate the pseudorange
    pr = distance + noise + clock_bias;
end

function range = generate_range(pos1, pos2, sigma)
    distance = norm(pos1 - pos2);

    noise = sigma * randn;
    
    range = distance + noise;
end


function sv = get_virtual_satellite(scenario, total_tle_number)
    % Ensure the number is between 1 and 4
    if total_tle_number < 1 || total_tle_number > 4
        error('tle_number must be between 1 and 4.');
    end

    sv = [];
    
    for tle_num = 1:total_tle_number
        % Dynamically generate the TLE file path based on the input number
        tle_file = sprintf('./data/tle%d.txt', tle_num);
        
        % Create the satellite object using the selected TLE file
        sv{end+1} = satellite(scenario, tle_file, "OrbitPropagator", "sgp4");
    end
end



function sv = get_gps_satellite(sceneario)
    sv = satellite(sceneario, './data/gpsalmanac.txt');
end