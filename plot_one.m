%% 초기 변수 설정 (기본값)
clear;

addpath('./module');
addpath('./helper');


folder_path = sprintf('./result/result_one');
if ~exist(folder_path, 'dir')
    mkdir(folder_path);
end

sigma_pr = 0.4;
sigma_range = 0.2;
r_sigma_pr = sigma_pr;
r_sigma_range = sigma_range;

%% 시뮬레이션
num_iterations = 400;
sv_num = 2;
convergence_idx = 250;

%% sigma에 따른 시뮬레이션
q_value = 5e-4;

dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2);
sv_pos = dataset.sat_positions;
sv_vel = dataset.sat_velocity;

sigma_range_list = [0.01, 0.1, 0.5];

total_pos = {};
total_vel = {};

for sigma_range = sigma_range_list
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2);
    [pr_range_pos, pr_range_vel] = run_simulation_with_pr_range(dataset, r_sigma_pr, sigma_range, q_value);
    total_pos{end+1} = pr_range_pos;
    total_vel{end+1} = pr_range_vel; 
end
[pr_pos, pr_vel] = run_simulation_with_pr(dataset, r_sigma_pr, q_value);
total_pos{end+1} = pr_pos;
total_vel{end+1} = pr_vel; 

ls_pos = run_simulation_with_ls_method(dataset);
total_pos{end+1} = ls_pos;

%% True Position 정의
true_position = [];
true_velocity = [];

target_sv = 1;

for i = convergence_idx:num_iterations
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
    true_velocity(:, i) = sv_vel{i, target_sv + 1} - sv_vel{i, target_sv};
end

%% 그래프 Plot

% 정확도 그래프 Plot
plot_pos_accuracy;

% 




%% Velocity error calculation
% Extract velocity estimates
estimated_velocity = pr_range_vel;  % Assuming velocity is in rows 4 to 6
estimated_velocity_with_pr = pr_vel;  % Velocity for KF without range
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