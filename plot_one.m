%% 초기 변수 설정 (기본값)
clear;





sigma_pr = 0.4;
sigma_range = 0.1;
r_sigma_pr = 0.4;
r_sigma_range = 0.1;

%% 시뮬레이션

result_folder = sprintf('./result/result_one');
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end





[kf_error_now, ls_error_now, kf_error_without_range_now] = main_abs_rel_range_tdcp(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range, result_folder, true);