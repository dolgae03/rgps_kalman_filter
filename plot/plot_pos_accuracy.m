
%% 3D RMS Error 계산
convergence_idx = 150;
end_num = 1000;

%% True Position 정의
true_position = [];
true_velocity = [];

target_sv = 1;

for i = convergence_idx:end_num
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
    true_velocity(:, i) = sv_vel{i, target_sv + 1} - sv_vel{i, target_sv};
end


time = convergence_idx:end_num;
total_error = {};

legend_strings = arrayfun(@(x) sprintf('EKF-ISL($\\sigma = %.2f$m)', x), sigma_range_list, 'UniformOutput', false);
legend_strings{end+1} = 'EKF-Pesudorange Only';
legend_strings{end+1} = 'LS';

% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
fig_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_pos.fig'));
txt_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_pos.txt'));
% 
% % 최종 RMS error 계산 및 출력
fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

for i=1:length(total_pos)
    estimated_position = total_pos{i};
    
    error_x = abs(true_position(1, convergence_idx:end_num) - estimated_position(1, convergence_idx:end_num));
    error_y = abs(true_position(2, convergence_idx:end_num) - estimated_position(2, convergence_idx:end_num));
    error_z = abs(true_position(3, convergence_idx:end_num) - estimated_position(3, convergence_idx:end_num));
    
    total_error{end+1} = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    fprintf(fileID, 'Final RMS error (%s) in X-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Y-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Z-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (%s) in 3D: %.4f meters\n', legend_strings{i}, sqrt(mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2).^2)));
end

fclose(fileID);

%% 그림 그리기 

fig = figure(4);  % 'Visible', 'off'로 설정하여 창을 띄우지 않음
clf;
fig.Color = "white";
hold on;

% 색상 맵 설정 (기본 'lines' 컬러 맵 사용)
colors = lines(length(total_error));  % sigma_idx 개의 다양한 색상 생성

% 실제 플롯
for i = 1:length(total_error)
    plot(time, total_error{i}, 'Color', colors(i, :), 'LineWidth', 2);  % 얇은 실제 선
    hold on;
end

% 범례에 표시하기 위한 더 굵은 선 (실제 플롯에 영향 없음)
for i = 1:length(total_error)
    p(i) = plot(nan, nan, 'Color', colors(i, :), 'LineWidth', 3);  % 굵은 선을 범례에만 추가
    hold on;
end

% 동적으로 생성된 legend 적용 및 위치 설정, LaTeX 해석을 사용
lgd = legend(p, legend_strings, 'Location', 'northeast', 'Interpreter', 'latex');  % 범례에 LaTeX 적용
set(lgd, 'FontSize', 24, 'FontWeight', 'bold');  % 범례 글꼴 크기와 두께 설정

% 축과 라벨의 글꼴 크기 및 두께 설정
set(gca, 'FontSize', 24);  % 축 글꼴 크기 및 두께 설정
xlabel('Time step', 'FontSize', 24, 'FontWeight', 'bold');  % X축 라벨 글꼴 크기 및 두께 설정
ylabel('Error (meters)', 'FontSize', 24, 'FontWeight', 'bold');  % Y축 라벨 글꼴 크기 및 두께 설정

xlim([convergence_idx, end_num])
grid on;
% FIG 파일 저장
savefig(fig, fig_file_pos_path);