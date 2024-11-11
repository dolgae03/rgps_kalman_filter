clear;

addpath('./module');
addpath('./helper');
addpath('./plot');

sigma_pr_values = 0.01:0.1:2;  % sigma_pr의 변화 범위
sigma_range_values = 0.01:0.05:1;  % sigma_range의 변화 범위

% 결과 폴더 설정
result_folder = './result/contour';  % 원하는 폴더 경로 설정
result_folder_detail = './result/contour/sub';  % 원하는 폴더 경로 설정
if ~exist(result_folder_detail, 'dir')
    mkdir(result_folder_detail);
end

%% 시뮬레이션s
num_iterations = 1000;
convergence_idx = 900;

constellation = 1;

sv_num = 2;

%% sigma에 따른 시뮬레이션
q_value = 1;

%% True Position 정의
true_position = [];

target_sv = 1;
dataset = make_dataset(num_iterations, 3, 3, 2, 'b', constellation);
sv_pos = dataset.sat_positions;
sv_vel = dataset.sat_velocity;

for i = convergence_idx:num_iterations
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
end



% 결과 저장을 위한 Z 행렬 초기화
kf_error = zeros(length(sigma_pr_values), length(sigma_range_values));
kf_cov_value = zeros(length(sigma_pr_values), length(sigma_range_values));


for i = 1:length(sigma_pr_values)
    for j = 1:length(sigma_range_values)
        sigma_pr = sigma_pr_values(i);
        sigma_range = sigma_range_values(j);

        dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', constellation);

        [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, sigma_pr, sigma_range, q_value);
        
        estimated_position = pr_range_pos;
    
        error_x = abs(true_position(1, convergence_idx:end) - estimated_position(1, convergence_idx:end));
        error_y = abs(true_position(2, convergence_idx:end) - estimated_position(2, convergence_idx:end));
        error_z = abs(true_position(3, convergence_idx:end) - estimated_position(3, convergence_idx:end));
        
        total_error = mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2));

        kf_cov_error(i, j) = total_error;

        kf_error(i, j) = total_error;

        x = cov(1, 1, :);
        y = cov(2, 2, :);
        z = cov(3, 3, :);

        res = mean(sqrt(x+y+z));
        
        kf_error(i, j) = res * 2;
    end
end

%% Figure 생성
clf;

fig = figure(151);
fig.Color = "white";

contour_levels_cov = linspace(min(kf_cov_error(:)), max(kf_cov_error(:)), 30);  % 20개 레벨로 설정
contourf(sigma_range_values, sigma_pr_values, kf_cov_error, contour_levels_cov);
colorbar;
xlabel('Standard Deviation ISL (m)', 'FontSize', 14, 'FontWeight', 'bold');  % Bold and font size increased
ylabel('Standard Deviation PR (m)', 'FontSize', 14, 'FontWeight', 'bold');
title('95% Accuracy (m)', 'FontSize', 14, 'FontWeight', 'bold'); % Bold title

% 축과 라벨의 글꼴 크기 및 두께 설정
set(gca, 'FontSize', 14);  % 축 글꼴 크기 및 두께 설정

% 그래프 저장
fig_file_name = sprintf('Contour_KF_vs_LS_Error_%s.fig', constellation);
fig_file_path = fullfile(result_folder, fig_file_name);
savefig(fig, fig_file_path);