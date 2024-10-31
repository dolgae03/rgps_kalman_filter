
%% 3D RMS Error 계산
convergence_idx = 1;
time = convergence_idx:num_iterations;
total_cov_plot = {};

legend_strings = {};
legend_strings{end+1} = 'Satellite 1' ;
legend_strings{end+1} = 'Satellite 2';
legend_strings{end+1} = 'Satellite 1, 2';

% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
fig_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_sat_num.fig'));

%% Visable Satellite Num 표시
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

fig = figure(6);  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
clf;
fig.Color = "white";
hold on;

% 색상 맵 설정 (기본 'lines' 컬러 맵 사용)
colors = lines(3);  % 3개의 다양한 색상 생성

% 선 스타일 설정
line_styles = {'-', '--', ':'};  % 각각의 위성에 대해 다른 선 스타일

% 실제 플롯
for i = 1:3
    stairs(time, visable_sat_mat(i, :), ...
           'Color', colors(i, :), ...
           'LineStyle', line_styles{i}, ...
           'LineWidth', 2);
    hold on;
end

% 범례에 표시하기 위한 더 굵은 선 (실제 플롯에 영향 없음)
for i = 1:3
    p(i) = plot(nan, nan, ...
                'Color', colors(i, :), ...
                'LineStyle', line_styles{i}, ...  % 범례에서도 동일한 선 스타일 적용
                'LineWidth', 3);  % 굵은 선을 범례에만 추가
    hold on;
end

% 동적으로 생성된 legend 적용 및 위치 설정, LaTeX 해석을 사용
lgd = legend(p, legend_strings, 'Location', 'northwest', 'Interpreter', 'latex');  % 범례에 LaTeX 적용
set(lgd, 'FontSize', 24, 'FontWeight', 'bold');  % 범례 글꼴 크기와 두께 설정

% 축과 라벨의 글꼴 크기 및 두께 설정
set(gca, 'FontSize', 24);  % 축 글꼴 크기 및 두께 설정
set(gca, 'YTick', 0:max(max(visable_sat_mat)) + 3);  
xlabel('Time step', 'FontSize', 24, 'FontWeight', 'bold');  % X축 라벨 글꼴 크기 및 두께 설정
ylabel('# of Satellite', 'FontSize', 24, 'FontWeight', 'bold');  % Y축 라벨 글꼴 크기 및 두께 설정

xlim([convergence_idx, num_iterations])
ylim([0, max(max(visable_sat_mat)) + 1])
grid on;

% FIG 파일 저장
savefig(fig, fig_file_pos_path);
