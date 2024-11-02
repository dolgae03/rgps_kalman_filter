%% 3D RMS Error 계산
convergence_idx = 200;
non_block_idx = 481;
end_num = 1000;

%% True Position 정의 BLOCKED
true_position = [];
true_velocity = [];

target_sv = 1;

for i = non_block_idx:end_num
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
    true_velocity(:, i) = sv_vel{i, target_sv + 1} - sv_vel{i, target_sv};
end

%% 관련 데이터 저장 방식 설정 

legend_strings = arrayfun(@(x) sprintf('EKF-ISL($\\sigma = %.2f$m)', x), sigma_range_list, 'UniformOutput', false);
legend_strings{end+1} = 'EKF-Pesudorange Only';
legend_strings{end+1} = 'LS';

total_error = {};
% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
txt_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_pos_blocked.txt'));
% 
% % 최종 RMS error 계산 및 출력
fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

for i=1:length(total_pos)
    estimated_position = total_pos{i};
    
    error_x = abs(true_position(1, non_block_idx:end_num) - estimated_position(1, non_block_idx:end_num));
    error_y = abs(true_position(2, non_block_idx:end_num) - estimated_position(2, non_block_idx:end_num));
    error_z = abs(true_position(3, non_block_idx:end_num) - estimated_position(3, non_block_idx:end_num));
    
    total_error{end+1} = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    fprintf(fileID, 'Final RMS error (%s) in X-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Y-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Z-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (%s) in 3D: %.4f meters\n', legend_strings{i}, sqrt(mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2).^2)));
end

fclose(fileID);

%% 관련 데이터 저장 방식 설정 NONBLOCKDED

true_position = [];
true_velocity = [];


total_error = {};

target_sv = 1;

for i = convergence_idx:non_block_idx
    true_position(:, i) = sv_pos{i, target_sv + 1} - sv_pos{i, target_sv};
    true_velocity(:, i) = sv_vel{i, target_sv + 1} - sv_vel{i, target_sv};
end

%% 관련 데이터 저장 방식 설정 

% %% File WKRTJD
% 
% % SIGMA_PR 및 SIGMA_RANGE 변수를 파일 이름에 포함시키기 위한 문자열 생성
txt_file_pos_path = fullfile(folder_path, ...
                            sprintf('result_pos_non_blocked.txt'));
% 
% % 최종 RMS error 계산 및 출력
fileID = fopen(txt_file_pos_path, 'w');  % 'w' 모드는 파일에 쓰기

for i=1:length(total_pos)
    estimated_position = total_pos{i};
    
    error_x = abs(true_position(1, convergence_idx:non_block_idx) - estimated_position(1, convergence_idx:non_block_idx));
    error_y = abs(true_position(2, convergence_idx:non_block_idx) - estimated_position(2, convergence_idx:non_block_idx));
    error_z = abs(true_position(3, convergence_idx:non_block_idx) - estimated_position(3, convergence_idx:non_block_idx));
    
    total_error{end+1} = sqrt(error_x.^2 + error_y.^2 + error_z.^2);

    fprintf(fileID, 'Final RMS error (%s) in X-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_x.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Y-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_y.^2)));
    fprintf(fileID, 'Final RMS error (%s) in Z-axis: %.4f meters\n', legend_strings{i}, sqrt(mean(error_z.^2)));
    fprintf(fileID, 'Final RMS error (%s) in 3D: %.4f meters\n', legend_strings{i}, sqrt(mean(sqrt(error_x.^2 + error_y.^2 + error_z.^2).^2)));
end

fclose(fileID);