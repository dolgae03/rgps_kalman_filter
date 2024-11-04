%% 초기 변수 설정 (기본값)
clear;

addpath('./module');
addpath('./helper');
addpath('./plot');


folder_path = sprintf('./result/result_one');
if ~exist(folder_path, 'dir')
    mkdir(folder_path);
end

sigma_pr = 1;
sigma_range = 0.2;
r_sigma_pr = sigma_pr;
r_sigma_range = sigma_range;

%% 시뮬레이션
num_iterations = 1000;
sv_num = 2;

%% sigma에 따른 시뮬레이션
dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', 0);
sv_pos = dataset.sat_positions;
sv_vel = dataset.sat_velocity;

sigma_range_list = [0.4];

q_value = 3;

total_pos = {};
total_vel = {};
total_cov = {};

for sigma_range = sigma_range_list
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', 0);
    [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, r_sigma_pr, sigma_range, q_value);
    total_pos{end+1} = pr_range_pos;
    total_vel{end+1} = pr_range_vel; 
    total_cov{end+1} = cov;
end
[pr_pos, pr_vel, cov] = run_simulation_with_pr(dataset, r_sigma_pr, q_value);
total_pos{end+1} = pr_pos;
total_vel{end+1} = pr_vel; 
total_cov{end+1} = cov;

total_pos_multi = {};
total_vel_multi = {};
total_cov_multi = {};

for sigma_range = sigma_range_list
    dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', 1);
    [pr_range_pos, pr_range_vel, cov] = run_simulation_with_pr_range(dataset, r_sigma_pr, sigma_range, q_value);
    total_pos_multi{end+1} = pr_range_pos;
    total_vel_multi{end+1} = pr_range_vel; 
    total_cov_multi{end+1} = cov;
end
[pr_pos, pr_vel, cov] = run_simulation_with_pr(dataset, r_sigma_pr, q_value);
total_pos_multi{end+1} = pr_pos;
total_vel_multi{end+1} = pr_vel; 
total_cov_multi{end+1} = cov;


% [ls_pos, cov] = run_simulation_with_ls_method(dataset);
% total_pos{end+1} = ls_pos;
% total_cov{end+1} = cov;

%% 그래프 Plot
% 위성의 Position Accuracy
save_pos_accuracy;

% 대충 그리기 
plot_true_position;

% 정확도 그래프 Plot (Position)
plot_pos_accuracy;

% 정확도 그래프 Plot (Sigma)
plot_cov;

% 위성 개수 그래프 Plot (Sigma)
plot_sat_num;

% 위성 Velocity Plot
plot_vel;

