% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements
function [kf_error_vec, ls_error_vec, kf_error_with_pr_vec] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range, folder_path, visable)
    addpath('./module');
    addpath('./helper');
    
    rng(42)
    
    %% 시뮬레이션 데이터 추출
    val_num = 16;

    num_iterations = 200; % 시간 단계 수
    convergence_idx = 15;
    
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range);
    true_position = (dataset.sat2_positions - dataset.sat1_positions);
    true_velocity = (dataset.sat2_velocity - dataset.sat1_velocity);
    
    % 데이터를 저장할 배열
    p_idx = 1;
    curr_time = [];
    pr_mes = dataset.pr_mes;
    range_mes = dataset.range_mes;
    carrier_mes = dataset.carrier_mes;
    gps_pos = dataset.gps_positions;
    estimated_states = zeros(val_num, num_iterations);
    estimated_P = zeros(val_num, val_num,  num_iterations);
    estimated_states_with_pr = zeros(val_num, num_iterations);
    ls_position = zeros(4, num_iterations);

    

    %% 초기 상태 (x, y, z, vx, vy, vz, b, b_dot)
    inital_P_sigma = 20;

    init_x = zeros(val_num, 1);
    init_x(1:3, 1) = dataset.sat1_positions(:, 1) + [randn; randn; randn] .* inital_P_sigma;
    init_x(4:6, 1) = dataset.sat1_velocity(:, 1) + [randn; randn; randn] .* inital_P_sigma;
    init_x(7, 1) = 3 + randn * inital_P_sigma;

    init_x(9:11, 1) = (dataset.sat2_positions(:, 1) - dataset.sat1_positions(:, 1)) + [randn; randn; randn] .* inital_P_sigma;
    init_x(12:14, 1) = dataset.sat2_velocity(:, 1) - dataset.sat1_velocity(:, 1)  + [randn; randn; randn] .* inital_P_sigma;
    init_x(15, 1) = randn * inital_P_sigma;

    P = inital_P_sigma * eye(val_num);
    
    %% Kalman Filter 정의
    use_external_force = false;
    
    kalman_filter = TC_ABS_REL_KF(init_x, P, use_external_force);
    kalman_filter_without_range = TC_ABS_REL_KF(init_x, P, use_external_force);
    
    %% Simulatin 수행
    for k = 1:num_iterations
        %% Prediction 단계
        
        dt = 1;
        for update_idx = 1:1/dt
            Q = 0.5 * eye(val_num);
            kalman_filter = kalman_filter.predict(Q, 1);

            Q_without_range = 3 * eye(val_num);
            kalman_filter_without_range = kalman_filter_without_range.predict(Q_without_range, dt);
    
            curr_time(p_idx) = k + update_idx * dt;
            estimated_P(:, :, p_idx) = kalman_filter.covariance;
            p_idx = p_idx + 1;
        end

        
        %% Correction 단계
        measurement_size = size(pr_mes{1, k}, 1);
        range_mes_size = size(range_mes, 1);
        carrier_mes_size = size(carrier_mes, 1);
        
        %% 전체 Kalman filter
        % Define Measurement Noise
        R = zeros(measurement_size * 2 + range_mes_size);
        R(1:measurement_size, 1:measurement_size) = r_sigma_pr .* eye(measurement_size);
        R(measurement_size + 1 : 2*measurement_size, measurement_size + 1 : 2*measurement_size) = r_sigma_pr * 2 .* eye(measurement_size);
        R(2*measurement_size + 1 : measurement_size * 2 + range_mes_size, 2*measurement_size + 1 : measurement_size * 2 + range_mes_size) = r_sigma_range .* eye(range_mes_size);
        % R(measurement_size * 2 + range_mes_size + 1 : end, measurement_size * 2 + range_mes_size + 1:end) = 5 * eye(3*2);

        R_only_pr = zeros(measurement_size * 2);
        R_only_pr(1:measurement_size, 1:measurement_size) = r_sigma_pr .* eye(measurement_size);
        R_only_pr(measurement_size + 1 : 2*measurement_size, measurement_size + 1 : 2*measurement_size) = r_sigma_pr * 2 .* eye(measurement_size);
    
        %% LS Measurement Update
        mes1 = pr_mes{1, k};
        mes2 = pr_mes{2, k};

        % carrier1_prev = carrier_mes{1, k-1};
        % carrier1_curr = carrier_mes{1, k};
        % carrier2_prev = carrier_mes{2, k-1};
        % carrier2_curr = carrier_mes{2, k};
          
        gps_pos_k = gps_pos{1, k};
        ref_pos = GNSS_LS(mes1, length(mes1), gps_pos_k);
        ref_pos2 = GNSS_LS(mes2,length(mes2), gps_pos_k);
    
        %% KF Measruement 처리
        z_abs = mes1;
        z_rel = mes1 - mes2;
        z_range = range_mes(:, 1, k);
        % z_tdcp = calculate_tdcp(kalman_filter.state(1:3, 1), kalman_filter.state(4:6, 1), 1, ...
        %                         carrier1_prev, gps_pos{1, k-1}', carrier1_curr, gps_pos_k);
    
        rotated_gps_pos_k_1 = zeros(size(gps_pos_k));
        rotated_gps_pos_k_2 = zeros(size(gps_pos_k));
        for j = 1:length(gps_pos_k)
            rotated_gps_pos_k_1(:,j) = rotate_gps_forward(gps_pos_k(:, j), kalman_filter.state(1:3, 1));
            rotated_gps_pos_k_2(:,j) = rotate_gps_forward(gps_pos_k(:, j), kalman_filter.state(1:3, 1) + kalman_filter.state(9:11, 1));
        end
        
        kalman_filter_without_range = kalman_filter_without_range.correct(rotated_gps_pos_k_1, rotated_gps_pos_k_2, z_abs, z_rel, [], R_only_pr);
        kalman_filter = kalman_filter.correct(rotated_gps_pos_k_1, rotated_gps_pos_k_2, z_abs, z_rel, z_range, R);
        
        %% 현재 Kalman_filter
        ls_position(:, k) = ref_pos2 - ref_pos;
        estimated_states(:, k) = kalman_filter.state;
        estimated_states_with_pr(:, k) = kalman_filter_without_range.state;

        curr_time(p_idx) = k+1;
        estimated_P(:, :, p_idx) = kalman_filter.covariance;
        p_idx = p_idx + 1;
    end
    
    
    
    
    %% 3D RMS Error 계산
    time = convergence_idx:num_iterations;
    estimated_position = estimated_states(9:11, :);
    ls_position_rel = ls_position(1:3, :);
    
    error_x = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
    error_y = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
    error_z = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));

    error_3d = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    %% kalman filter 에러 계산
    estimated_position = estimated_states_with_pr(9:11, :);
    error_x_with_pr = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
    error_y_with_pr = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
    error_z_with_pr = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));

    error_3d_with_pr = sqrt(error_x_with_pr.^2 + error_y_with_pr.^2 + error_z_with_pr.^2);

    %% LS Position 에러 계산
    ls_error_x = abs(true_position(1, convergence_idx:end) - ls_position_rel(1, convergence_idx:end));
    ls_error_y = abs(true_position(2, convergence_idx:end) - ls_position_rel(2, convergence_idx:end));
    ls_error_z = abs(true_position(3, convergence_idx:end) - ls_position_rel(3, convergence_idx:end));

    ls_error_3d = sqrt(ls_error_x.^2 + ls_error_y.^2 + ls_error_z.^2);
    
    
    % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
    fig_file_vel_path = fullfile(folder_path, ...
                                sprintf('result_pos_%0.3f_%0.3f_%0.3f_%0.3f.fig', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range));
    fig_file_pos_path = fullfile(folder_path, ...
                                sprintf('result_vel_%0.3f_%0.3f_%0.3f_%0.3f.fig', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range));

    txt_file_vel_path = fullfile(folder_path, ...
                                sprintf('result_vel_%0.3f_%0.3f_%0.3f_%0.3f.txt', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range));
    txt_file_pos_path = fullfile(folder_path, ...
                                sprintf('result_pos_%0.3f_%0.3f_%0.3f_%0.3f.txt', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range));

    % 최종 RMS error 계산 및 출력
    fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

    % 결과를 파일에 저장
    fprintf(fileID, 'Final RMS error (KF) in X-axis: %.4f meters\n', sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (KF) in Y-axis: %.4f meters\n', sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (KF) in Z-axis: %.4f meters\n', sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (KF) in 3D: %.4f meters\n', sqrt(mean(error_3d.^2)));
    
    fprintf(fileID, 'Final RMS error (KF without range) in X-axis: %.4f meters\n', sqrt(mean(error_x_with_pr.^2)));
    fprintf(fileID, 'Final RMS error (KF without range) in Y-axis: %.4f meters\n', sqrt(mean(error_y_with_pr.^2)));
    fprintf(fileID, 'Final RMS error (KF without range) in Z-axis: %.4f meters\n', sqrt(mean(error_z_with_pr.^2)));
    fprintf(fileID, 'Final RMS error (KF without range) in 3D: %.4f meters\n', sqrt(mean(error_3d_with_pr.^2)));

    fprintf(fileID, 'Final RMS error (LS) in X-axis: %.4f meters\n', sqrt(mean(ls_error_x.^2)));
    fprintf(fileID, 'Final RMS error (LS) in Y-axis: %.4f meters\n', sqrt(mean(ls_error_y.^2)));
    fprintf(fileID, 'Final RMS error (LS) in Z-axis: %.4f meters\n', sqrt(mean(ls_error_z.^2)));
    fprintf(fileID, 'Final RMS error (LS) in 3D: %.4f meters\n', sqrt(mean(ls_error_3d.^2)));

    kf_error_vec = [sqrt(mean(error_x.^2)); sqrt(mean(error_y.^2)); sqrt(mean(error_z.^2)); sqrt(mean(error_3d.^2))];
    kf_error_with_pr_vec = [sqrt(mean(error_x_with_pr.^2)); sqrt(mean(error_y_with_pr.^2)); sqrt(mean(error_z_with_pr.^2)); sqrt(mean(error_3d_with_pr.^2))];
    ls_error_vec = [sqrt(mean(ls_error_x.^2)); sqrt(mean(ls_error_y.^2)); sqrt(mean(ls_error_z.^2)); sqrt(mean(ls_error_3d.^2))];
    
    % 파일 닫기
    fclose(fileID);
    
    % 서브플롯 생성
    if ~visable
        return;
    end

    fig = figure(4);  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
    clf;
    hold on;
    
    subplot(4,1,1);
    plot(time, error_x, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_x_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_x, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range','LS');
    title('X-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,2);
    plot(time, error_y, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_y_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_y, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range','LS');
    title('Y-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,3);
    plot(time, error_z, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_z_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_z, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range','LS');
    title('Z-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,4);
    plot(time, error_3d, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_3d_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_3d, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range','LS');
    title('3D Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    % FIG 파일 저장
    savefig(fig, fig_file_pos_path);


    %% Velocity error calculation
    % Extract velocity estimates
    estimated_velocity = estimated_states(12:14, :);  % Assuming velocity is in rows 4 to 6
    estimated_velocity_with_pr = estimated_states_with_pr(12:14, :);  % Velocity for KF without range
    % ls_velocity_rel = ls_velocity(1:3, :);  % LS estimated velocity
    
    % Calculate velocity errors
    error_vx = abs(true_velocity(1, convergence_idx:end) - estimated_velocity(1, convergence_idx:end));
    error_vy = abs(true_velocity(2, convergence_idx:end) - estimated_velocity(2, convergence_idx:end));
    error_vz = abs(true_velocity(3, convergence_idx:end) - estimated_velocity(3, convergence_idx:end));
    
    error_v_3d = sqrt(error_vx.^2 + error_vy.^2 + error_vz.^2);
    
    error_vx_with_pr = abs(true_velocity(1, convergence_idx:end) - estimated_velocity_with_pr(1, convergence_idx:end));
    error_vy_with_pr = abs(true_velocity(2, convergence_idx:end) - estimated_velocity_with_pr(2, convergence_idx:end));
    error_vz_with_pr = abs(true_velocity(3, convergence_idx:end) - estimated_velocity_with_pr(3, convergence_idx:end));
    
    error_v_3d_with_pr = sqrt(error_vx_with_pr.^2 + error_vy_with_pr.^2 + error_vz_with_pr.^2);
    
    % % LS Velocity error
    % ls_error_vx = abs(true_velocity(1, convergence_idx:end) - ls_velocity_rel(1, convergence_idx:end));
    % ls_error_vy = abs(true_velocity(2, convergence_idx:end) - ls_velocity_rel(2, convergence_idx:end));
    % ls_error_vz = abs(true_velocity(3, convergence_idx:end) - ls_velocity_rel(3, convergence_idx:end));
    % 
    % ls_error_v_3d = sqrt(ls_error_vx.^2 + ls_error_vy.^2 + ls_error_vz.^2);
    
    fileID = fopen(txt_file_vel_path, 'w');  % 'w' 모드는 파일에 쓰기

    % Final RMS velocity error calculation
    fprintf(fileID, 'Final RMS velocity error (KF) in X-axis: %.4f meters/sec\n', sqrt(mean(error_vx.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF) in Y-axis: %.4f meters/sec\n', sqrt(mean(error_vy.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF) in Z-axis: %.4f meters/sec\n', sqrt(mean(error_vz.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF) in 3D: %.4f meters/sec\n', sqrt(mean(error_v_3d.^2)));
    
    fprintf(fileID, 'Final RMS velocity error (KF without range) in X-axis: %.4f meters/sec\n', sqrt(mean(error_vx_with_pr.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF without range) in Y-axis: %.4f meters/sec\n', sqrt(mean(error_vy_with_pr.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF without range) in Z-axis: %.4f meters/sec\n', sqrt(mean(error_vz_with_pr.^2)));
    fprintf(fileID, 'Final RMS velocity error (KF without range) in 3D: %.4f meters/sec\n', sqrt(mean(error_v_3d_with_pr.^2)));
    
    % fprintf(fileID, 'Final RMS velocity error (LS) in X-axis: %.4f meters/sec\n', sqrt(mean(ls_error_vx.^2)));
    % fprintf(fileID, 'Final RMS velocity error (LS) in Y-axis: %.4f meters/sec\n', sqrt(mean(ls_error_vy.^2)));
    % fprintf(fileID, 'Final RMS velocity error (LS) in Z-axis: %.4f meters/sec\n', sqrt(mean(ls_error_vz.^2)));
    % fprintf(fileID, 'Final RMS velocity error (LS) in 3D: %.4f meters/sec\n', sqrt(mean(ls_error_v_3d.^2)));

    fclose(fileID);
    
    % Subplot for velocity error
    fig1 = figure(1);
    clf;
    hold on;

    subplot(4,1,1);
    plot(time, error_vx, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_vx_with_pr, '-g', 'LineWidth', 1);
    % plot(time, ls_error_vx, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range');
    title('X-axis Velocity Error over Time');
    xlabel('Time step');
    ylabel('Error (m/s)');
    grid on;
    
    subplot(4,1,2);
    plot(time, error_vy, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_vy_with_pr, '-g', 'LineWidth', 1);
    % plot(time, ls_error_vy, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range');
    title('Y-axis Velocity Error over Time');
    xlabel('Time step');
    ylabel('Error (m/s)');
    grid on;
    
    subplot(4,1,3);
    plot(time, error_vz, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_vz_with_pr, '-g', 'LineWidth', 1);
    % plot(time, ls_error_vz, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range');
    title('Z-axis Velocity Error over Time');
    xlabel('Time step');
    ylabel('Error (m/s)');
    grid on;
    
    subplot(4,1,4);
    plot(time, error_v_3d, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_v_3d_with_pr, '-g', 'LineWidth', 1);
    % plot(time, ls_error_v_3d, '-b', 'LineWidth', 1);
    legend('EKF with range', 'EKF without range');
    title('3D Velocity Error over Time');
    xlabel('Time step');
    ylabel('Error (m/s)');
    grid on;
    
    % Save velocity error figure

    savefig(fig1, fig_file_vel_path);

    %% Draw P value of velocity
    extract_P_xx = zeros(val_num, num_iterations);
    for i = 1:length(curr_time)
        extract_P_xx(:, i) = diag(estimated_P(:, :, i));
    end

    draw_error(curr_time, extract_P_xx);

    %% Draw True Norm and 

    true_position_norm = vecnorm(true_position, 2, 1);  % Compute the norm across time steps for true position
    estimated_states_norm = vecnorm(estimated_states(9:11, :), 2, 1);  % Compute the norm across time steps for estimated position
    
    fig = figure(2);
    clf;
    plot(1:num_iterations, true_position_norm - estimated_states_norm, '-r', 'LineWidth', 1, 'DisplayName', 'true norm');  % Plot true norm
    % hold on;
    % % plot(1:num_iterations, estimated_states_norm, '-b', 'LineWidth', 1, 'DisplayName', 'estimated norm');  % Plot estimated norm
     
    xlabel('Time step');
    ylabel('E (sigma square)');
    grid on;
    legend(); 

    %% Draw clock bias of LS and KF

    fig = figure(5);
    clf;

    plot(1:num_iterations, estimated_states(7, :), '-r', 'LineWidth', 1, 'DisplayName', 'clock bias KF');  % Plot true norm
    hold on;
    plot(1:num_iterations,estimated_states_with_pr(7, :), '-b', 'LineWidth', 1, 'DisplayName', 'clock bias KF with out range');  % Plot estimated norm
     
    xlabel('Time step');
    ylabel('E (sigma square)');
    grid on;
    legend(); 


end


function draw_error(time, xyz)
    fig2 = figure(3);
    clf;
    hold on;

    % 색상 배열 (각 성분에 대해 다른 색을 사용할 수 있도록)
    styles = generate_styles;  % 필요에 따라 더 많은 색을 추가 가능

    % xyz의 모든 성분을 for문으로 플롯
    for i = 1:size(xyz, 1)
        style = styles{mod(i-1, length(styles))+1};  % 스타일 순환 적용
        plot(time, xyz(i, :), style, 'LineWidth', 1, 'DisplayName',sprintf('xyz(%d)', i));  % 스타일 적용하여 플롯
        hold on;
    end

    legend(arrayfun(@(i) sprintf('xyz(%d)', i), 1:size(xyz, 1), 'UniformOutput', false));


    % x, y축 라벨과 그리드 설정
    xlabel('Time step');
    ylabel('Covariance (sigma square)');
    grid on;
end


function styles = generate_styles()
    % 색상 배열
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];  % 7개의 색상
    
    % 점 스타일 배열
    markers = ['.', 'o', 'x', '+', '*', 's', 'd', 'v'];  % 8개의 점 스타일
    
    % 선 타입 배열
    line_styles = {'-', '--', ':', '-.'};  % 4개의 선 타입
    
    % 16개의 조합을 저장할 셀 배열
    styles = cell(1, 16);
    
    % 색상, 점 스타일, 선 타입을 조합하여 스타일 생성
    idx = 1;
    for i = 1:length(colors)
        for k = 1:length(line_styles)
            if idx > 16
                break;  % 16개 조합을 초과하면 종료
            end
            % 색상, 선 타입, 점 스타일을 결합하여 스타일 생성
            styles{idx} = [colors(i), line_styles{k}];
            idx = idx + 1;
        end
    end
end