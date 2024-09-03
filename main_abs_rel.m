% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements
addpath('./module');

%% 초기 상태 (x, y, z, vx, vy, vz, b, b_dot)
val_num = 16;

init_x = zeros(val_num, 1); 
P = 1000 * eye(val_num);

%% 시뮬레이션 데이터 추출
num_iterations = 150; % 시간 단계 수
sigma = 0;
convergence_idx = 50;

dataset = make_dataset(num_iterations, sigma);
true_position = (dataset.sat2_positions - dataset.sat1_positions)';

% 데이터를 저장할 배열
measurements = dataset.measurements;
gps_pos = dataset.gps_positions;
estimated_states = zeros(val_num, num_iterations);
ls_position = zeros(4, num_iterations);

%% Kalman Filter 정의
kalman_filter = TC_ABS_REL_KF(init_x, P);

%% Simulatin 수행
for k = 1:num_iterations
    %% Prediction 단계
    Q = 0.1 * eye(val_num);
    kalman_filter = kalman_filter.predict(Q, 1);
    
    %% Correction 단계
    R = 20 .* eye(size(measurements, 1) * 2);

    mes = measurements(:, :, k);
    gps_pos_k = gps_pos(:, :, k)';

    z_abs = measurements(:, 1, k);
    z_rel = measurements(:, 1, k) - measurements(:, 2, k);

    ref_pos = GNSS_LS(measurements(:, 1, k), length(mes), gps_pos_k);
    ref_pos2 = GNSS_LS(measurements(:, 2, k),length(mes), gps_pos_k);
    
    kalman_filter = kalman_filter.correct(gps_pos_k, z_abs, z_rel, R);
    
    % 추정된 상태 저장
    ls_position(:, k) = ref_pos2 - ref_pos;
    estimated_states(:, k) = kalman_filter.state;
end


%% 3D RMS Error 계산
estimated_position = estimated_states(9:11, :);
ls_position_rel = ls_position(1:3, :);

% 결과 플로팅
time = convergence_idx:num_iterations;

error_x = true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end);
error_y = true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end);
error_z = true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end);

error_3d = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

% LS Position 에러 계산
ls_error_x = true_position(1, convergence_idx:end) - ls_position(1, convergence_idx:end);
ls_error_y = true_position(2, convergence_idx:end) - ls_position(2, convergence_idx:end);
ls_error_z = true_position(3, convergence_idx:end) - ls_position(3, convergence_idx:end);
ls_error_3d = sqrt(ls_error_x.^2 + ls_error_y.^2 + ls_error_z.^2);

%% 서브플롯 생성
figure;

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

% 최종 RMS error 계산 및 출력
fprintf('Final RMS error (KF) in X-axis: %.4f meters\n', sqrt(mean(error_x.^2)));
fprintf('Final RMS error (KF) in Y-axis: %.4f meters\n', sqrt(mean(error_y.^2)));
fprintf('Final RMS error (KF) in Z-axis: %.4f meters\n', sqrt(mean(error_z.^2)));
fprintf('Final RMS error (KF) in 3D: %.4f meters\n', sqrt(mean(error_3d.^2)));

fprintf('Final RMS error (LS) in X-axis: %.4f meters\n', sqrt(mean(ls_error_x.^2)));
fprintf('Final RMS error (LS) in Y-axis: %.4f meters\n', sqrt(mean(ls_error_y.^2)));
fprintf('Final RMS error (LS) in Z-axis: %.4f meters\n', sqrt(mean(ls_error_z.^2)));
fprintf('Final RMS error (LS) in 3D: %.4f meters\n', sqrt(mean(ls_error_3d.^2)));