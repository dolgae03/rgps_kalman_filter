function dataset = make_dataset(num_samples, sigma_pr, sigma_range)
    rng(42)

    start_time = datetime(2024, 9, 8, 14, 30, 0);

    times = start_time + seconds((1:num_samples));

    pr_mes = cell(2, num_samples);
    carrier_mes = cell(2, num_samples);
    range_mes = zeros(2, 1, num_samples);

    [gps_pos, sv1_pos, sv2_pos, sv1_vel, sv2_vel, gps_visablility] = make_position_data(start_time, num_samples);
    
    for i = 1:num_samples
        pos1 = sv1_pos(:, i);
        pos2 = sv2_pos(:, i);

        pos_gps = gps_pos{i};
        for j = 1:size(pos_gps, 2)
            pos_gps_when_send_signal = rotate_gps_forward(pos_gps(:, j), pos1);
            pr_mes{1, i}(j,1) = generate_pr(pos_gps_when_send_signal, pos1, sigma_pr);
            carrier_mes{1, i}(j,1) = generate_carrier(pos_gps_when_send_signal, pos1, 0.02);

            pos_gps_when_send_signal = rotate_gps_forward(pos_gps(:, j), pos2);
            pr_mes{2, i}(j,1) = generate_pr(pos_gps_when_send_signal, pos2, sigma_pr);
            carrier_mes{2, i}(j,1) = generate_carrier(pos_gps_when_send_signal, pos2, 0.02);
        end

        range_mes(1, 1, i) = generate_range(pos1, pos2, sigma_range);
        range_mes(2, 1, i) = generate_range(pos1, pos2, sigma_range);
    end

    dataset.sat1_positions = sv1_pos;
    dataset.sat2_positions = sv2_pos;
    dataset.sat1_velocity = sv1_vel;
    dataset.sat2_velocity = sv2_vel;
    dataset.pr_mes = pr_mes;
    dataset.range_mes = range_mes;
    dataset.carrier_mes = carrier_mes;
    dataset.gps_positions = gps_pos;
    dataset.gps_visablity = gps_visablility;
    dataset.times = times;
end

function [gps_pos, sv1_pos, sv2_pos, sv1_vel, sv2_vel, gps_visablity] = make_position_data(start_time, num_samples)
    % 파일 이름 생성
    folder_name = './data/position_data';
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);  % data 폴더가 없으면 생성
    end
    % 날짜와 샘플 수 기반으로 파일 이름 생성
    file_name = sprintf('%s/position_velocity_data_%s_%d.mat', folder_name, datestr(start_time, 'yyyymmdd_HHMMSS'), num_samples);
    
    % 파일이 존재하는지 확인
    if exist(file_name, 'file')
        % 파일이 존재하면 데이터를 로드
        disp('파일이 존재합니다. 데이터를 로드합니다...');
        loaded_data = load(file_name);
        gps_pos = loaded_data.gps_pos;
        sv1_pos = loaded_data.sv1_pos;
        sv2_pos = loaded_data.sv2_pos;
        sv1_vel = loaded_data.sv1_vel;
        sv2_vel = loaded_data.sv2_vel;
        gps_visablity = loaded_data.gps_visablity;
    else
        % 파일이 존재하지 않으면 데이터를 생성
        disp('파일이 존재하지 않습니다. 데이터를 생성합니다...');
        stop_time = start_time + seconds(num_samples);

        % 위성 시나리오 생성
        sc = satelliteScenario(start_time, stop_time, num_samples, 'AutoSimulate', false);
        
        % 가상 위성 1, 2를 얻음
        [sv1, sv2] = get_virtual_satellite(sc);

        % GPS 위성을 얻음
        gps_sv = get_gps_satellite(sc)';

        % 시뮬레이션 시간 계산
        times = start_time + seconds((1:num_samples));

        v = satelliteScenarioViewer(sc,'ShowDetails',true);
        play(sc);


        % 위성들의 위치 및 속도 데이터를 저장할 변수 초기화
        sv1_pos = zeros(3, num_samples);
        sv2_pos = zeros(3, num_samples);
        sv1_vel = zeros(3, num_samples);
        sv2_vel = zeros(3, num_samples);
        gps_pos = cell(1, num_samples); % GPS 위성 위치 저장
        gps_visablity = zeros(3, num_samples);

        % Elevation Threshold
        elevation_threshold = 10;

        for i = 1:num_samples
            % 각 시점에서 가상 위성 1과 2의 위치를 가져옴
            [pos1, vel1] = states(sv1, times(i), 'CoordinateFrame', 'ecef');
            [pos2, vel2] = states(sv2, times(i), 'CoordinateFrame', 'ecef');

            % GPS 위성의 위치를 가져옴
            pos_gps = states(gps_sv, times(i), 'CoordinateFrame', 'ecef');

            idx = 1;

            % 각 GPS 위성에 대해 Elevation 각도 체크 후 위치 계산
            for j = 1:31
                if calculate_elevation(pos1, pos_gps(:, 1, j)) > elevation_threshold
                    gps_visablity(1, i) = gps_visablity(1, i)  + 1
                end

                if calculate_elevation(pos2, pos_gps(:, 1, j)) > elevation_threshold
                    gps_visablity(2, i) = gps_visablity(2, i)  + 1
                end
              

                if calculate_elevation(pos1, pos_gps(:, 1, j)) > elevation_threshold ...
                   && calculate_elevation(pos2, pos_gps(:, 1, j)) > elevation_threshold
           
                    % GPS 위성 위치 저장
                    gps_pos{1, i}(:, idx) = pos_gps(:, 1, j);
                    idx = idx + 1;

                    gps_visablity(3, i) = gps_visablity(3, i)  + 1
                end
            end

            % 가상 위성 1과 2의 위치와 속도 저장
            sv1_pos(:, i) = pos1; % X, Y, Z 좌표
            sv2_pos(:, i) = pos2; % X, Y, Z 좌표
            sv1_vel(:, i) = vel1;      % X, Y, Z 속도
            sv2_vel(:, i) = vel2;      % X, Y, Z 속도
        end

        % 데이터를 파일로 저장
        save(file_name, 'gps_pos', 'sv1_pos', 'sv2_pos', 'sv1_vel', 'sv2_vel', 'gps_visablity');
        disp(['데이터가 저장되었습니다: ' file_name]);
    end
end

function pr = generate_carrier(gps_pos, sat_pos, sigma)
    distance = norm(gps_pos - sat_pos);

    % Generate Gaussian noise with mean 0 and standard deviation sigma
    noise = sigma * randn;
    N = 12315;
    
    % Calculate the pseudorange
    pr = distance + noise;
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


function [sv1, sv2] = get_virtual_satellite(scenario)
    sv1 = satellite(scenario, './data/tle1.txt', ...
                    "OrbitPropagator","sgp4");
    sv2 = satellite(scenario, './data/tle2.txt', ...
                    "OrbitPropagator","sgp4");

end

function sv = get_gps_satellite(sceneario)
    sv = satellite(sceneario, './data/gpsalmanac.txt');
end