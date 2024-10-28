% Kalman Filter with 3D RMS Error Plot for both KF solution and Measurements
function ls_result = run_simulation_with_ls_method(dataset)
    addpath('./module');
    addpath('./helper');
    
    rng(42)

    %% 초기 변수 정의
    SV_NUM = 2;
    
    % 데이터를 저장할 배열
    pr_mes = dataset.pr_mes;
    gps_pos = dataset.gps_positions;

    ls_position = zeros(4, length(pr_mes));
    
    %% Simulatin 시작
    for i = 1:length(pr_mes)
        for k = 1:SV_NUM-1
            valid_indices_pr = ~isnan(pr_mes{i, k}) & ~isnan(pr_mes{i, k+1});

            mes1 = pr_mes{i, k}(valid_indices_pr);
            mes2 = pr_mes{i, k+1}(valid_indices_pr);

            gps_pos_k = gps_pos{i, k}(:, valid_indices_pr);

            ref_pos = GNSS_LS(mes1, length(mes1), gps_pos_k);
            ref_pos2 = GNSS_LS(mes2,length(mes2), gps_pos_k);
            ls_position(:, i) = ref_pos2 - ref_pos;
        end
    end
    

    ls_result = ls_position;
end
