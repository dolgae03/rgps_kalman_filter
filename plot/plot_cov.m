
%% 3D RMS Error 계산
convergence_idx = 1;
time = convergence_idx:num_iterations;
total_cov_plot = {};

legend_strings = arrayfun(@(x) sprintf('EKF-ISL($\\sigma = %.2f$m)', x), sigma_range_list, 'UniformOutput', false);
legend_strings{end+1} = 'EKF-Pesudorange Only';
legend_strings{end+1} = 'LS';

% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
fig_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_sigma.fig'));

txt_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_sigma.txt'));
% 
% % 최종 RMS error 계산 및 출력
fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

for i=1:length(total_cov)
    estimated_cov = total_cov{i};
    
    estimated_position = []
    for j=1:size(estimated_cov, 3)
        estimated_position(:,j) = diag(estimated_cov(1:3, 1:3, j));
    end
    
    error_x = sqrt(estimated_position(1, convergence_idx:end));
    error_y = sqrt(estimated_position(2, convergence_idx:end));
    error_z = sqrt(estimated_position(3, convergence_idx:end));
    
    total_cov_plot{end+1} = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    fprintf(fileID, 'Final RMS sigma (%s) in X-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in Y-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in Z-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in 3D: %.4f meters\n', legend_strings{i}, sqrt(mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2).^2)));
end

fclose(fileID);

%% 그림 그리기 

fig = figure(5);  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
clf;
fig.Color = "white";
hold on;

% 색상 맵 설정 (기본 'lines' 컬러 맵 사용)
colors = lines(length(total_cov_plot));  % sigma_idx 개의 다양한 색상 생성

% 실제 플롯
for i = 1:length(total_cov_plot)
    plot(time, total_cov_plot{i}, 'Color', colors(i, :), 'LineWidth', 1);  % 얇은 실제 선
    hold on;
end

% 범례에 표시하기 위한 더 굵은 선 (실제 플롯에 영향 없음)
for i = 1:length(total_cov_plot)
    p(i) = plot(nan, nan, 'Color', colors(i, :), 'LineWidth', 3);  % 굵은 선을 범례에만 추가
    hold on;
end

% 동적으로 생성된 legend 적용 및 위치 설정, LaTeX 해석을 사용
lgd = legend(p, legend_strings, 'Location', 'northwest', 'Interpreter', 'latex');  % 범례에 LaTeX 적용
set(lgd, 'FontSize', 24, 'FontWeight', 'bold');  % 범례 글꼴 크기와 두께 설정

% 축과 라벨의 글꼴 크기 및 두께 설정
set(gca, 'FontSize', 24);  % 축 글꼴 크기 및 두께 설정
xlabel('Time step', 'FontSize', 24, 'FontWeight', 'bold');  % X축 라벨 글꼴 크기 및 두께 설정
ylabel('Error (meters)', 'FontSize', 24, 'FontWeight', 'bold');  % Y축 라벨 글꼴 크기 및 두께 설정

xlim([convergence_idx, num_iterations])
grid on;
% FIG 파일 저장
savefig(fig, fig_file_pos_path);