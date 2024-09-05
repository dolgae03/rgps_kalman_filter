% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements
function [kf_error_vec, ls_error_vec] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range, folder_path, visable)
    addpath('./module');
    addpath('./helper');
    
    rng(42)
    
    %% 초기 상태 (x, y, z, vx, vy, vz, b, b_dot)
    val_num = 16;
    
    init_x = zeros(val_num, 1); 
    init_x(9:11, 1) = [3; 3; 3];
    
    
    P = 1000 * eye(val_num);
    
    %% 시뮬레이션 데이터 추출
    num_iterations = 150; % 시간 단계 수
    convergence_idx = 50;
    
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range);
    true_position = (dataset.sat2_positions - dataset.sat1_positions)';
    
    % 데이터를 저장할 배열
    pr_mes = dataset.pr_mes;
    range_mes = dataset.range_mes;
    gps_pos = dataset.gps_positions;
    estimated_states = zeros(val_num, num_iterations);
    ls_position = zeros(4, num_iterations);
    
    %% Kalman Filter 정의
    kalman_filter = TC_ABS_REL_KF(init_x, P, false);
    
    %% Simulatin 수행
    for k = 1:num_iterations
        %% Prediction 단계
        Q = 1 * eye(val_num);
        kalman_filter = kalman_filter.predict(Q, 1);
        
        %% Correction 단계
        measurement_size = size(pr_mes, 1);
        range_mes_size = size(range_mes, 1);
    
        % Define Measurement Noise
        R = zeros(measurement_size * 2 + range_mes_size);
        R(1:measurement_size, 1:measurement_size) = r_sigma_pr .* eye(measurement_size);
        R(measurement_size + 1 : 2*measurement_size, measurement_size + 1 : 2*measurement_size) = r_sigma_pr * 2 .* eye(measurement_size);
        R(2*measurement_size + 1 : end, 2*measurement_size + 1 : end) = r_sigma_range .* eye(range_mes_size);
    
        % LS Measurement Update
        mes = pr_mes(:, :, k);
        gps_pos_k = gps_pos(:, :, k)';
    
        ref_pos = GNSS_LS(pr_mes(:, 1, k), length(mes), gps_pos_k);
        ref_pos2 = GNSS_LS(pr_mes(:, 2, k),length(mes), gps_pos_k);
    
        % KF Measruement 처리
        z_abs = pr_mes(:, 1, k);
        z_rel = pr_mes(:, 1, k) - pr_mes(:, 2, k);
        z_range = range_mes(:, 1, k);
    
        rotated_gps_pos_k = zeros(size(gps_pos_k));
        for j = 1:length(gps_pos_k)
            rotated_gps_pos_k(:,j) = rotate_gps_forward(gps_pos_k(:, j), ref_pos(1:3, 1));
        end
        
        kalman_filter = kalman_filter.correct(rotated_gps_pos_k, z_abs, z_rel, z_range, R);
        
        % 추정된 상태 저장
        ls_position(:, k) = ref_pos2 - ref_pos;
        estimated_states(:, k) = kalman_filter.state;
    end
    
    
    
    
    %% 3D RMS Error 계산
    estimated_position = estimated_states(9:11, :);
    ls_position_rel = ls_position(1:3, :);
    
    % 결과 플로팅
    time = convergence_idx:num_iterations;
    
    error_x = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
    error_y = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
    error_z = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));

    error_3d = sqrt(error_x.^2 + error_y.^2 + error_z.^2);
    
    % LS Position 에러 계산
    ls_error_x = abs(true_position(1, convergence_idx:end) - ls_position_rel(1, convergence_idx:end));
    ls_error_y = abs(true_position(2, convergence_idx:end) - ls_position_rel(2, convergence_idx:end));
    ls_error_z = abs(true_position(3, convergence_idx:end) - ls_position_rel(3, convergence_idx:end));

    ls_error_3d = sqrt(ls_error_x.^2 + ls_error_y.^2 + ls_error_z.^2);
    
    
    % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
    fig_file_name = sprintf('result_%0.3f_%0.3f_%0.3f_%0.3f.fig', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range);
    fig_file_path = fullfile(folder_path, fig_file_name);

    txt_file_name = sprintf('result_%0.3f_%0.3f_%0.3f_%0.3f.txt', sigma_pr, sigma_range, r_sigma_pr, r_sigma_range);
    txt_file_path = fullfile(folder_path, txt_file_name);

    % 최종 RMS error 계산 및 출력
    fileID = fopen(txt_file_path, 'w');  % 'w' 모드는 파일에 쓰기

    % 결과를 파일에 저장
    fprintf(fileID, 'Final RMS error (KF) in X-axis: %.4f meters\n', sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (KF) in Y-axis: %.4f meters\n', sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (KF) in Z-axis: %.4f meters\n', sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (KF) in 3D: %.4f meters\n', sqrt(mean(error_3d.^2)));
    
    fprintf(fileID, 'Final RMS error (LS) in X-axis: %.4f meters\n', sqrt(mean(ls_error_x.^2)));
    fprintf(fileID, 'Final RMS error (LS) in Y-axis: %.4f meters\n', sqrt(mean(ls_error_y.^2)));
    fprintf(fileID, 'Final RMS error (LS) in Z-axis: %.4f meters\n', sqrt(mean(ls_error_z.^2)));
    fprintf(fileID, 'Final RMS error (LS) in 3D: %.4f meters\n', sqrt(mean(ls_error_3d.^2)));

    kf_error_vec = [sqrt(mean(error_x.^2)); sqrt(mean(error_y.^2)); sqrt(mean(error_z.^2)); sqrt(mean(error_3d.^2))];
    ls_error_vec = [sqrt(mean(ls_error_x.^2)); sqrt(mean(ls_error_y.^2)); sqrt(mean(ls_error_z.^2)); sqrt(mean(ls_error_3d.^2))];
    
    % 파일 닫기
    fclose(fileID);
    
    % 서브플롯 생성
    if ~visable
        return;
    end

    fig = figure();  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
    hold on;
    
    subplot(4,1,1);
    plot(time, error_x, '-r', 'LineWidth', 2);
    hold on;
    plot(time, ls_error_x, '--r', 'LineWidth', 2);
    legend('KF X-axis Error', 'LS X-axis Error');
    title('X-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,2);
    plot(time, error_y, '-g', 'LineWidth', 2);
    hold on;
    plot(time, ls_error_y, '--g', 'LineWidth', 2);
    legend('KF Y-axis Error', 'LS Y-axis Error');
    title('Y-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,3);
    plot(time, error_z, '-b', 'LineWidth', 2);
    hold on;
    plot(time, ls_error_z, '--b', 'LineWidth', 2);
    legend('KF Z-axis Error', 'LS Z-axis Error');
    title('Z-axis Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,4);
    plot(time, error_3d, '-k', 'LineWidth', 2);
    hold on;
    plot(time, ls_error_3d, '--k', 'LineWidth', 2);
    legend('KF 3D Error', 'LS 3D Error');
    title('3D Error over Time');
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    % FIG 파일 저장
    savefig(fig, fig_file_path);

    close(fig);
end