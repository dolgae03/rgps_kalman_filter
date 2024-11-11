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

constellation = 0;

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


kf_error_file = fullfile(result_folder, 'kf_error_data.mat');
if exist(kf_error_file, 'file')
    % Load precomputed kf_error data
    load(kf_error_file, 'kf_error_1', 'kf_error_2');
else
    % Run the computation loop
    kf_error_1 = zeros(length(sigma_pr_values), length(sigma_range_values));
    kf_error_2 = zeros(length(sigma_pr_values), length(sigma_range_values));
    
    for i = 1:length(sigma_pr_values)
        for j = 1:length(sigma_range_values)
            sigma_pr = sigma_pr_values(i);
            sigma_range = sigma_range_values(j);

            dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', 0);
            [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, sigma_pr, sigma_range, q_value);

            x = cov(1, 1, :);
            y = cov(2, 2, :);
            z = cov(3, 3, :);
            res = mean(sqrt(x + y + z));
            kf_error_1(i, j) = res * 2;

            dataset = make_dataset(num_iterations, sigma_pr, 500000000, 2, 'b', 1);
            [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, sigma_pr, 500000000, q_value);

            x = cov(1, 1, :);
            y = cov(2, 2, :);
            z = cov(3, 3, :);
            res = mean(sqrt(x + y + z));
            kf_error_2(i, j) = res * 2;
        end
    end

    % Save the computed results
    save(kf_error_file, 'kf_error_1', 'kf_error_2');
end


%% Figure 생성
clf;

% Initialize figure
fig = figure(1);
fig.Color = 'white';

% Use tiledlayout for better control over layout and colorbar
t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Create shared color scale based on the combined min and max values
min_val = min([kf_error_1(:); kf_error_2(:)]);  % Include both datasets
max_val = max([kf_error_1(:); kf_error_2(:)]);
contour_levels = 0:0.25:4;  % 30 levels for consistency

% First contour plot (left side)
nexttile;
contourf(sigma_range_values, sigma_pr_values, kf_error_1, contour_levels);
xlabel('Standard Deviation ISL (m)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Standard Deviation PR (m)', 'FontSize', 14, 'FontWeight', 'bold');
title('95% Accuracy (GPS) (m)', 'FontSize', 14, 'FontWeight', 'bold');
caxis([0 4])
set(gca, 'FontSize', 14);

% Second contour plot (right side)
nexttile;
contourf(sigma_range_values, sigma_pr_values, kf_error_2, contour_levels);
xlabel('Standard Deviation ISL (m)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Standard Deviation PR (m)', 'FontSize', 14, 'FontWeight', 'bold');
title('95% Accuracy (GPS+GAL) (m)', 'FontSize', 14, 'FontWeight', 'bold');
caxis([0 4])
set(gca, 'FontSize', 14);

% Add colorbar to the figure instead of the layout
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Ticks = contour_levels;

% Save figure
fig_file_name = sprintf('Contour_KF_vs_LS_Error.fig');
fig_file_path = fullfile(result_folder, fig_file_name);
savefig(fig, fig_file_path);
