
%% 3D RMS Error 계산
convergence_idx = 1;
time = convergence_idx:num_iterations;
total_cov_plot = {};
total_cov_plot_multi = {};
legend_strings = {};

legend_strings{end+1} = sprintf('ISL/RGPS(GPS+GAL)');
legend_strings{end+1} = 'RGPS Only(GPS+GAL)';
legend_strings{end+1} = 'Visable GPS+GAL';

% legend_strings{end+1} = 'LS';

% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
fig_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_sigma_gal.fig'));

txt_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_sigma_gal.txt'));

% % 최종 RMS error 계산 및 출력
fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

for i=1:2
    estimated_cov = total_cov_multi{i};
    
    estimated_position = []
    for j=1:size(estimated_cov, 3)
        estimated_position(:,j) = diag(estimated_cov(1:3, 1:3, j));
    end
    
    error_x = sqrt(estimated_position(1, convergence_idx:end)) * 2;
    error_y = sqrt(estimated_position(2, convergence_idx:end)) * 2;
    error_z = sqrt(estimated_position(3, convergence_idx:end)) * 2;
    
    total_cov_plot{end+1} = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    fprintf(fileID, 'Final RMS sigma (%s) in X-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in Y-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in Z-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS sigma (%s) in 3D: %.4f meters\n', legend_strings{i}, sqrt(mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2).^2)));

end


fclose(fileID);

dataset = make_dataset(num_iterations, sigma_pr, sigma_range, 2, 'b', 1);
pr_mes = dataset.pr_mes;
visable_sat_mat = [];

idx = 1;

for i = time
    visable_sat_mat(1, idx) = sum(~isnan(pr_mes{i, 1}));
    visable_sat_mat(2, idx) = sum(~isnan(pr_mes{i, 2}));
    visable_sat_mat(3, idx) = sum(~isnan(pr_mes{i, 1}) & ~isnan(pr_mes{i, 2}));
    idx = idx + 1;
end

%% 그림 그리기 

fig = figure(10);  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
clf;
fig.Color = "white";
hold on;

% 색상 맵 설정 (기본 'lines' 컬러 맵 사용)
colors = [0.49,0.18,0.56;
          0.93,0.69,0.13];  % sigma_idx 개의 다양한 색상 생성

% 왼쪽 Y축: Standard Deviation
yyaxis left
for i = 1:length(total_cov_plot)
    h(i) = plot(time, total_cov_plot{i}, 'Color', colors(i, :), 'LineStyle', '-', 'Marker', 'none','LineWidth', 2);  % 실선 스타일, 마커 없음
    hold on;
end

% 범례에 표시하기 위한 더 굵은 선 (실제 플롯에 영향 없음)
for i = 1:length(total_cov_plot)
    p_cov_gal(i) = plot(nan, nan, 'Color', colors(i, :), 'LineStyle', '-', 'Marker', 'none', 'LineWidth', 2);  % 범례용 굵은 실선, 마커 없음
    hold on;
end

p_cov_gal(end+1) = plot(nan, nan, 'Color', [1.00,0.00,0.00], 'LineStyle', '--','Marker', 'none', 'LineWidth', 2);  % 범례용 굵은 실선, 마커 없음
    



% 왼쪽 Y축 라벨 설정
ylabel('95% Accuracy (m)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');  
ylim([1, 8])
set(gca, 'YColor', 'k');  %

% 오른쪽 Y축: Sat Num
yyaxis right
stairs(time, visable_sat_mat(3, :), 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);  % 실선 스타일, 마커 없음
hold on;
ylabel('# of Satellite', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');  % 오른쪽 Y축 라벨 (빨간색)


% 축과 라벨의 글꼴 크기 및 두께 설정
set(gca, 'FontSize', 14);  % 축 글꼴 크기 및 두께 설정
xlabel('Time step (s)', 'FontSize', 14, 'FontWeight', 'bold');  % X축 라벨 글꼴 크기 및 두께 설정

% 동적으로 생성된 legend 적용 및 위치 설정, LaTeX 해석을 사용
lgd = legend(p_cov_gal, legend_strings, 'Location', 'northeast', 'Interpreter', 'latex');  % 범례에 LaTeX 적용
set(lgd, 'FontSize', 14, 'FontWeight', 'bold');  % 범례 글꼴 크기와 두께 설정
set(gca, 'YColor', 'r');  %


% X축 제한
xlim([convergence_idx, num_iterations])
ylim([3.5, 36])

grid on;

% FIG 파일 저장
savefig(fig, fig_file_pos_path);
