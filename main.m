% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements

%% 초기 상태 (x, y, z, vx, vy, vz, b, b_dot)
x = [0; 0; 0; 0;]; % 초기 상태 추정

P = [1000 * eye(3) zeros(3,1);
     zeros(1,3) 1]; % 초기 공분산 추정

%% 시뮬레이션 설정
num_iterations = 4000; % 시간 단계 수

dataset = make_dataset(num_iterations);
% true_position = (dataset.sat2_positions - dataset.sat1_positions)';
true_position = (dataset.sat1_positions)';

true_bias = 0; % 시계 오차

% 데이터를 저장할 배열
measurements = dataset.measurements;
gps_pos = dataset.gps_positions;
estimated_states = zeros(4, num_iterations);
ls_position = zeros(4, num_iterations, 2);

%% True Position 생성 및 Measurement 생성
for k = 1:num_iterations
    
    % Prediction 단계
    % [x, P] = prediction(x, P, A, Q);
    
    % Correction 단계
    R = 0.1.* eye(size(measurements, 1));

    mes = measurements(:, :, k);
    gps_pos_k = gps_pos(:, :, k)';

    ref_pos = GNSS_LS(mes(:,1), length(mes), gps_pos_k);
    ref_pos2 = GNSS_LS(mes(:, 2),length(mes), gps_pos_k);

    [x, P] = correction(x, P, gps_pos_k, mes, R);
    
    % 추정된 상태 저장
    ls_position(:, k, 1) = ref_pos;
    ls_position(:, k, 2) = ref_pos2;
    estimated_states(:, k) = x;
end

%% 3D RMS Error 계산
estimated_position = estimated_states(1:3, :);
kf_errors = sqrt(sum((true_position - estimated_position(1:3, :)).^2, 1));
kf_rms_error = sqrt(mean(kf_errors.^2));
ls_position_rel = ls_position(1:3, k, 2) - ls_position(1:3, k, 1);

% 결과 플로팅
convergence_idx = 3800;
time = convergence_idx:num_iterations;

error_x = true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end);
error_y = true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end);
error_z = true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end);

error_3d = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

%% 서브플롯 생성
figure;

subplot(4,1,1);
plot(time, error_x, '-r', 'LineWidth', 2);
legend('X-axis Error');
title('X-axis Error over Time');
xlabel('Time step');
ylabel('Error (meters)');
grid on;

subplot(4,1,2);
plot(time, error_y, '-g', 'LineWidth', 2);
legend('Y-axis Error');
title('Y-axis Error over Time');
xlabel('Time step');
ylabel('Error (meters)');
grid on;

subplot(4,1,3);
plot(time, error_z, '-b', 'LineWidth', 2);
legend('Z-axis Error');
title('Z-axis Error over Time');
xlabel('Time step');
ylabel('Error (meters)');
grid on;

subplot(4,1,4);
plot(time, error_3d, '-k', 'LineWidth', 2);
legend('3D Error');
title('3D Error over Time');
xlabel('Time step');
ylabel('Error (meters)');
grid on;

% 최종 RMS error 계산 및 출력
fprintf('Final RMS error (KF) in X-axis: %.4f meters\n', sqrt(mean(error_x.^2)));
fprintf('Final RMS error (KF) in Y-axis: %.4f meters\n', sqrt(mean(error_y.^2)));
fprintf('Final RMS error (KF) in Z-axis: %.4f meters\n', sqrt(mean(error_z.^2)));
fprintf('Final RMS error (KF) in 3D: %.4f meters\n', sqrt(mean(error_3d.^2)));