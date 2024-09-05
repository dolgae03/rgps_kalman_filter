% 초기 변수 설정 (기본값)
r_sigma_pr = 5;
r_sigma_range = 0.1;% 초기 변수 범위 설정 (변화 범위 지정)

sigma_pr_values = 1:1:4;  % sigma_pr의 변화 범위
sigma_range_values = 0.1:0.5:2;  % sigma_range의 변화 범위

% 결과 폴더 설정
result_folder = './result/contour';  % 원하는 폴더 경로 설정
result_folder_detail = './result/contour/sub';  % 원하는 폴더 경로 설정
if ~exist(result_folder_detail, 'dir')
    mkdir(result_folder_detail);
end

% 결과 저장을 위한 Z 행렬 초기화
kf_error = zeros(length(sigma_range_values), length(sigma_pr_values));
ls_error = zeros(length(sigma_range_values), length(sigma_pr_values));

% 2중 for문을 사용하여 Z 값 계산
for i = 1:length(sigma_range_values)
    for j = 1:length(sigma_pr_values)
        sigma_pr = sigma_pr_values(j);
        sigma_range = sigma_range_values(i);
        
        [kf_error_now, ls_error_now] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range, result_folder_detail, false);
        % 실제 문제에 맞는 함수 계산 (여기서는 예시로 sqrt 사용)
        % 원하는 함수로 변경하여 Z 값 계산
        kf_error(i, j) = kf_error_now(4);
        ls_error(i, j) = ls_error_now(4);
    end
end

%% Figure 생성
fig = figure;

% 첫 번째 subplot - KF Error contour plot
subplot(1, 2, 1);
contour(sigma_pr_values, sigma_range_values, kf_error, 'ShowText', 'on');
colorbar;
xlabel('\sigma_{pr}', 'FontSize', 12);
ylabel('\sigma_{range}', 'FontSize', 12);
title('Contour of KF Error', 'FontSize', 14);

% 두 번째 subplot - LS Error contour plot
subplot(1, 2, 2);
contour(sigma_pr_values, sigma_range_values, ls_error, 'ShowText', 'on');
colorbar;
xlabel('\sigma_{pr}', 'FontSize', 12);
ylabel('\sigma_{range}', 'FontSize', 12);
title('Contour of LS Error', 'FontSize', 14);

% 그래프 저장
fig_file_name = 'Contour_KF_vs_LS_Error.fig';
fig_file_path = fullfile(result_folder, fig_file_name);
savefig(fig, fig_file_path);

% 그래프 닫기
close(fig);