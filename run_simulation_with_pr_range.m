% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements
function [pos_sol, vel_sol, cov_sol] = run_simulation_with_pr_range(dataset, r_sigma_pr, r_sigma_range, q_value)
    addpath('./module');
    addpath('./helper');
    
    rng(42)

    %% 초기 변수 정의
    SV_NUM = 2;
    val_num = 16;

    use_external_force = true;
    
    % 데이터를 저장할 배열
    sv_pos = dataset.sat_positions;
    sv_vel = dataset.sat_velocity;
    pr_mes = dataset.pr_mes;
    range_mes = dataset.range_mes;
    gps_pos = dataset.gps_positions;

    %% 초기 상태 칼만필터 생성 및 초기값 정의 (x, y, z, vx, vy, vz, b, b_dot)

    kalman_filter_list = cell(SV_NUM, 1);

    inital_P_sigma = 20;

    for k = 1:SV_NUM-1
        init_x = zeros(val_num, 1);
        init_x(1:3, 1) = sv_pos{1,k}(:, 1) + [randn; randn; randn] .* inital_P_sigma;
        init_x(4:6, 1) = sv_vel{1,k}(:, 1) + [randn; randn; randn] .* inital_P_sigma;
        init_x(7, 1) = 3 + randn * inital_P_sigma;
        
        init_x(9:11, 1) = (sv_pos{1,k+1}(:, 1) - sv_pos{1,k}(:, 1)) + [randn; randn; randn] .* inital_P_sigma;
        init_x(12:14, 1) = (sv_vel{1,k+1}(:, 1) - sv_vel{1,k}(:, 1))  + [randn; randn; randn] .* inital_P_sigma;
        init_x(15, 1) = randn * inital_P_sigma;
        
        init_P = inital_P_sigma * eye(val_num);
        
        kalman_filter_list{k} = TC_TDCP_KF(init_x, init_P, use_external_force);
    end
    
    %% Simulatin 시작
    for i = 1:length(sv_pos)
        %% Prediction
        for k = 1:SV_NUM-1
            dt = 1;
            for update_idx = 1:1/dt
                Q = q_value * eye(val_num);
                kalman_filter_list{k} = kalman_filter_list{k}.predict(Q, 1);
            end
        end

        
        for k = 1:SV_NUM-1
            valid_indices_pr = ~isnan(pr_mes{i, k}) & ~isnan(pr_mes{i, k+1});

            mes1 = pr_mes{i, k}(valid_indices_pr);
            mes2 = pr_mes{i, k+1}(valid_indices_pr);
            r_mes = range_mes{i, k};

            gps_pos_k = gps_pos{i, k}(:, valid_indices_pr);
            %% Define Measurement Noise
            measurement_size = size(mes1, 1);
            range_mes_size = size(r_mes, 1);
    
            R = zeros(measurement_size * 2 + range_mes_size);
            R(1:measurement_size, 1:measurement_size) = r_sigma_pr .* eye(measurement_size);
            R(measurement_size + 1 : 2*measurement_size, measurement_size + 1 : 2*measurement_size) = r_sigma_pr * 2 .* eye(measurement_size);
            R(2*measurement_size + 1 : measurement_size * 2 + range_mes_size, 2*measurement_size + 1 : measurement_size * 2 + range_mes_size) = r_sigma_range .* eye(range_mes_size);

            %% KF Measruement 처리
            z_abs = mes1;
            z_rel = mes1 - mes2;
            z_range = r_mes;

            %% KF Measurement 처리
            rotated_gps_pos_k_1 = zeros(size(gps_pos_k));
            rotated_gps_pos_k_2 = zeros(size(gps_pos_k));
            for j = 1:length(gps_pos_k)
                rotated_gps_pos_k_1(:,j) = rotate_gps_forward(gps_pos_k(:, j), kalman_filter_list{k}.state(1:3, 1));
                rotated_gps_pos_k_2(:,j) = rotate_gps_forward(gps_pos_k(:, j), kalman_filter_list{k}.state(1:3, 1) + kalman_filter_list{k}.state(9:11, 1));
            end

            kalman_filter_list{k} = kalman_filter_list{k}.inital_correct(rotated_gps_pos_k_1, rotated_gps_pos_k_2, z_abs, z_rel, z_range, R);
        end
    end
    
    pos_sol = kalman_filter_list{1}.inital_log(9:11, :);
    vel_sol = kalman_filter_list{1}.inital_log(12:14, :);
    cov_sol = kalman_filter_list{1}.cov_log(9:11, 9:11, 2:end);
end