% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements

% 초기 설정
dt = 1; % 시간 간격
A = [1 0 0 dt 0 0 0 0; % x
     0 1 0 0 dt 0 0 0; % y
     0 0 1 0 0 dt 0 0; % z
     0 0 0 1 0 0 0 0; % vx
     0 0 0 0 1 0 0 0; % vy
     0 0 0 0 0 1 0 0; % vz
     0 0 0 0 0 0 1 dt; % b
     0 0 0 0 0 0 0 1]; % b_dot

Q = diag([0.1 0.1 0.1 0.1 0.1 0.1 0.01 0.01]); % 프로세스 노이즈 공분산

%% 초기 상태 (x, y, z, vx, vy, vz, b, b_dot)
x = [30; 30; 30; 1; 1; 1; 0; 0]; % 초기 상태 추정
P = eye(8); % 초기 공분산 추정

%% 시뮬레이션 설정
num_iterations = 300; % 시간 단계 수


dataset = make_dataset(num_iterations);
true_position = (dataset.sat2_positions - dataset.sat1_positions)';
true_velocity = [1; 1; 1];

true_bias = 0; % 시계 오차
true_bias_drift = 0.00; % 시계 오차의 드리프트
true_state = [0; 0; 0; true_velocity; true_bias; true_bias_drift];

% 데이터를 저장할 배열
measurements = dataset.measurements;
estimated_states = zeros(8, num_iterations);

%% True Position 생성 및 Measurement 생성
for k = 1:num_iterations
    
    % Prediction 단계
    % [x, P] = prediction(x, P, A, Q);
    
    % Correction 단계
    R = 5.* eye(size(measurements, 1));
    [x, P] = correction(x, P, measurements(:, :, k), R);
    
    % 추정된 상태 저장
    estimated_states(:, k) = x;
end

% 3D RMS Error 계산
kf_errors = sqrt(sum((true_position - estimated_states(1:3, :)).^2, 1));
kf_rms_error = sqrt(mean(kf_errors.^2));

% 결과 플로팅
time = 1:num_iterations;

% 3D Trajectory Plot
figure;
plot3(true_position(1, :), true_position(2, :), true_position(3, :), '-g', 'LineWidth', 2); hold on;
plot3(estimated_states(1, :), estimated_states(2, :), estimated_states(3, :), '-r', 'LineWidth', 2);

legend('True Position', 'Estimated Position');
title('Kalman Filter Position Estimation (3D)');
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
grid on;

% RMS Error Plot
figure;
plot(time, kf_errors, '-r', 'LineWidth', 2);
legend('KF RMS Error');
title('3D RMS Error over Time');
xlabel('Time step');
ylabel('3D RMS Error');
grid on;

fprintf('Final RMS error (KF): %.4f meters\n', kf_rms_error);
