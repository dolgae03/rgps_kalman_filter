%% 초기 변수 설정 (기본값)
clear;

addpath('./module');
addpath('./helper');
addpath('./plot');


folder_path = sprintf('./result/result_one');
if ~exist(folder_path, 'dir')
    mkdir(folder_path);
end

sigma_pr = 0.4;
sigma_range = 0.2;
r_sigma_pr = sigma_pr;
r_sigma_range = sigma_range;

%% 시뮬레이션
num_iterations = 1000;
sv_num = 2;

%% sigma에 따른 시뮬레이션
q_value = 2e-2;

dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2);
sv_pos = dataset.sat_positions;
sv_vel = dataset.sat_velocity;

sigma_range_list = [0.01, 0.1, 0.5];

total_pos = {};
total_vel = {};
total_cov = {};

for sigma_range = sigma_range_list
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2);
    [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, r_sigma_pr, sigma_range, q_value);
    total_pos{end+1} = pr_range_pos;
    total_vel{end+1} = pr_range_vel; 
    total_cov{end+1} = cov;
end
[pr_pos, pr_vel, cov] = run_simulation_with_pr(dataset, r_sigma_pr, q_value);
total_pos{end+1} = pr_pos;
total_vel{end+1} = pr_vel; 
total_cov{end+1} = cov;

[ls_pos, cov] = run_simulation_with_ls_method(dataset);
total_pos{end+1} = ls_pos;
total_cov{end+1} = cov;

%% 그래프 Plot

% 정확도 그래프 Plot (Position)
plot_pos_accuracy;

% 정확도 그래프 Plot (Sigma)
plot_cov;

% 위성 개수 그래프 Plot (Sigma)
plot_sat_num;