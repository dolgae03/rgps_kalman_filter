function analyze_results()
    %% True Position 정의
    true_position = [];
    true_velocity = [];
    target_sv = 2;

    for i = convergence_idx:num_iterations
        true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
        true_velocity(:, i) = sv_vel{i, target_sv + 1} - sv_vel{i, target_sv};
    end
    
    %% 3D RMS Error 계산
    time = convergence_idx:num_iterations;
    estimated_position = kalman_filter_list{target_sv}.final_log(9:11, :);
    ls_position_rel = ls_position(1:3, :);
    
    error_x = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
    error_y = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
    error_z = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));

    error_3d = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    %% kalman filter 에러 계산
    estimated_position = kalman_filter_without_final_update_list{target_sv}.inital_log(9:11, :);
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
    fprintf(fileID, 'Final RMS error (KF with State Update) in X-axis: %.4f meters\n', sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (KF with State Update) in Y-axis: %.4f meters\n', sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (KF with State Update) in Z-axis: %.4f meters\n', sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (KF with State Update) in 3D: %.4f meters\n', sqrt(mean(error_3d.^2)));
    
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
    legend('EKF final', 'EKF inital','LS');
    title('X-axis Error over Time');
    xlim([convergence_idx, num_iterations])
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,2);
    plot(time, error_y, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_y_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_y, '-b', 'LineWidth', 1);
    legend('EKF final', 'EKF inital','LS');
    title('Y-axis Error over Time');
    xlim([convergence_idx, num_iterations])
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    subplot(4,1,3);
    plot(time, error_z, '-r', 'LineWidth', 1);
    hold on;
    plot(time, error_z_with_pr, '-g', 'LineWidth', 1);
    hold on;
    plot(time, ls_error_z, '-b', 'LineWidth', 1);
    legend('EKF final', 'EKF inital','LS');
    xlim([convergence_idx, num_iterations])
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
    legend('EKF final', 'EKF inital','LS');
    title('3D Error over Time');
    xlim([convergence_idx, num_iterations])
    xlabel('Time step');
    ylabel('Error (meters)');
    grid on;
    
    % FIG 파일 저장
    savefig(fig, fig_file_pos_path);


    %% Velocity error calculation
    % Extract velocity estimates
    estimated_velocity = kalman_filter_list{target_sv}.final_log(12:14, :);  % Assuming velocity is in rows 4 to 6
    estimated_velocity_with_pr = kalman_filter_without_final_update_list{target_sv}.inital_log(12:14, :);  % Velocity for KF without range
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
    xlim([convergence_idx, num_iterations])
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
    xlim([convergence_idx, num_iterations])
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
    xlim([convergence_idx, num_iterations])
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
    xlim([convergence_idx, num_iterations])
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

    %% GPS visable
    figure(6);
    clf;
    
    % 첫 번째 데이터 시리즈 (gps_visable의 첫 번째 행)
    plot(1:num_iterations, gps_visable(1,:), '--r', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', 'Data Point 1');  
    hold on;
    
    % 두 번째 데이터 시리즈 (gps_visable의 두 번째 행)
    plot(1:num_iterations, gps_visable(2,:), ':g', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', 'Data Point 2');  
    
    % 세 번째 데이터 시리즈 (gps_visable의 세 번째 행)
    plot(1:num_iterations, gps_visable(3,:), '-.b', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', 'Data Point 3');  
    
    xlabel('Time step');
    ylabel('Visible Satellite Number');
    grid on;
    legend();
    hold off;



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