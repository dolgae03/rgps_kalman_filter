% 초기 변수 설정 (기본값)
sigma_pr = 3;
sigma_range = 0.01;
r_sigma_pr = 3;
r_sigma_range = 0.1;

%% sigma_pr 동적으로 변화시키는 for문
idx = 1;
kf_error = [];
ls_error = [];

range = 0.1:0.5:5;
for sigma_pr_dynamic = range
    % 폴더 이름 생성
    result_folder = sprintf('./result/result_by_pr');
    if ~exist(result_folder, 'dir')
        mkdir(result_folder);
    end
    
    % 동적 값과 기본값을 이용한 함수 호출
    [kf_error_now, ls_error_now] = main_abs_rel_range(sigma_pr_dynamic, sigma_range, r_sigma_pr, r_sigma_range, result_folder, false);
    kf_error(:, idx) = kf_error_now;
    ls_error(:, idx) = ls_error_now;
    idx = idx + 1;
end

plot_change_graph(range, ls_error, kf_error, result_folder)

%% sigma_range 동적으로 변화시키는 for문
idx = 1;
kf_error = [];
ls_error = [];

range = 0.01:0.01:0.1;
for sigma_range_dynamic = range
    % 폴더 이름 생성
    result_folder = sprintf('./result/result_range');
    if ~exist(result_folder, 'dir')
        mkdir(result_folder);
    end
    
    % 동적 값과 기본값을 이용한 함수 호출
    [kf_error_now, ls_error_now] = main_abs_rel_range(sigma_pr, sigma_range_dynamic, r_sigma_pr, r_sigma_range, result_folder, false);
    kf_error(:, idx) = kf_error_now;
    ls_error(:, idx) = ls_error_now;
    idx = idx + 1;
end

plot_change_graph(range, ls_error, kf_error, result_folder)


%% 몰라..

idx = 1;
kf_error = [];
ls_error = [];

range = 1:0.3:3;
for r_sigma_pr_dynamic = range
    result_folder = sprintf('./result/result_r_sigma_pr');
    if ~exist(result_folder, 'dir')
        mkdir(result_folder);
    end
    
    [kf_error_now, ls_error_now] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr_dynamic, r_sigma_range, result_folder, false);
    kf_error(:, idx) = kf_error_now;
    ls_error(:, idx) = ls_error_now;
    idx = idx + 1;
end

plot_change_graph(range, ls_error, kf_error, result_folder)

%% r_sigma_range 동적으로 변화시키는 for문
idx = 1;
kf_error = [];
ls_error = [];

range = 0.1:0.05:5;
for r_sigma_range_dynamic = range
    % 폴더 이름 생성
    result_folder = sprintf('./result/result_r_sigma_range');
    if ~exist(result_folder, 'dir')
        mkdir(result_folder);
    end
    
    [kf_error_now, ls_error_now] = main_abs_rel_range(sigma_pr, sigma_range, r_sigma_pr, r_sigma_range_dynamic, result_folder, false);
    kf_error(:, idx) = kf_error_now;
    ls_error(:, idx) = ls_error_now;
    idx = idx + 1;
end


function plot_change_graph(sigma_range, ls_error, kf_error, folder_path)
    % Check if folder exists, if not, create it
    if ~exist(folder_path, 'dir')
        mkdir(folder_path);
    end
    
    % 각 축에 대한 레이블 설정
    axis_labels = {'X-axis', 'Y-axis', 'Z-axis', '3D'};

    % Figure 생성
    fig = figure();
    
    % Loop over 4 dimensions (x, y, z, 3d) and create subplots
    for i = 1:4
        subplot(4, 1, i);
        hold on;
        
        % 각 에러에 대해 플롯
        plot(sigma_range, ls_error(i, :), '-og', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'LS Error');
        plot(sigma_range, kf_error(i, :), '--db', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'KF Error');
        
        % 범례 추가
        legend('show', 'Location', 'NorthWest');
        
        % 축 레이블 및 제목 추가
        xlabel('\sigma', 'FontSize', 12);
        ylabel(sprintf('The RMSE of %s', axis_labels{i}), 'FontSize', 12);
        title(sprintf('The RMSE of %s vs. Sigma', axis_labels{i}), 'FontSize', 14);
        
        % 그리드 및 축 범위 설정
        grid on;
    end
    
    % 그래프를 FIG 파일로 저장
    fig_file_name = 'RMSE_vs_Sigma_All_Axes.fig';
    fig_file_path = fullfile(folder_path, fig_file_name);
    savefig(fig, fig_file_path);
    
    % 그래프를 닫음
    close(fig);
end