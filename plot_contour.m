clear;

sigma_pr_values = 0.01:0.3:1;  % sigma_pr의 변화 범위
sigma_range_values = 0.01:0.2:0.4;  % sigma_range의 변화 범위

% 결과 폴더 설정
result_folder = './result/contour';  % 원하는 폴더 경로 설정
result_folder_detail = './result/contour/sub';  % 원하는 폴더 경로 설정
if ~exist(result_folder_detail, 'dir')
    mkdir(result_folder_detail);
end

%% 시뮬레이션s
num_iterations = 1000;
convergence_idx = 300;
sv_num = 2;

%% sigma에 따른 시뮬레이션
q_value = 1;

%% True Position 정의
true_position = [];

target_sv = 1;
dataset = make_dataset(num_iterations, 3, 3, 2);
sv_pos = dataset.sat_positions;
sv_vel = dataset.sat_velocity;

for i = convergence_idx:num_iterations
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
end



% 결과 저장을 위한 Z 행렬 초기화
kf_error = zeros(length(sigma_pr_values), length(sigma_range_values));


for i = 1:length(sigma_pr_values)
    for j = 1:length(sigma_range_values)
        sigma_pr = sigma_pr_values(i);
        sigma_range = sigma_range_values(j);

        dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2);

        [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, sigma_pr, sigma_range, q_value);
        
        estimated_position = pr_range_pos;
    
        error_x = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
        error_y = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
        error_z = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));
        
        total_error = mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2));

        kf_error(i, j) = total_error;
    end
end

%% Figure 생성
clf;

fig = figure(1);

% 첫 번째 subplot - KF Error contour plot
contourf(sigma_pr_values, sigma_range_values, kf_error', 'ShowText', 'on');  % 등고선 간격을 0.1로 설정
colorbar;
xlabel('\sigma_{pr}', 'FontSize', 12);
ylabel('\sigma_{range}', 'FontSize', 12);
title('Contour of KF Error', 'FontSize', 14);


% 그래프 저장
fig_file_name = 'Contour_KF_vs_LS_Error.fig';
fig_file_path = fullfile(result_folder, fig_file_name);
savefig(fig, fig_file_path);
