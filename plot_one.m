% 초기 변수 설정 (기본값)
clear;

sigma_pr = 5;
sigma_range = 0.001;
r_sigma_pr = 8;
r_sigma_range = 0.001;

result_folder = sprintf('./result/result_one');
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

[kf_error_now, ls_error_now, kf_error_without_range_now] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range, result_folder, true);