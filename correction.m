function [x_corr, P_corr] = correction(x_pred, P_pred, sv_pos, measurement, R)
    % x_pred: 예측된 상태 추정값 (Predicted State Estimate)
    % P_pred: 예측된 공분산 추정값 (Predicted Covariance Estimate)
    % z: 실제 측정값 (Measurement)
    % H: 측정 행렬 (Measurement Matrix)
    % R: 측정 노이즈 공분산 (Measurement Noise Covariance)

    % 칼만 이득 계산 (Kalman Gain)

    ref_pos = GNSS_LS(measurement(:,1), length(measurement), sv_pos);
    H = compute_H_absolute(sv_pos, ref_pos, x_pred);

    K = P_pred * H' * inv(H * P_pred * H' + R);

    % z = measurement(:, 1) - measurement(:, 2);
    z = measurement(:, 1);
    % 상태 수정 (State Correction)
    x_corr = x_pred + K * (z - h_absolute(sv_pos, ref_pos, x_pred));
    
    % 공분산 수정 (Covariance Correction)
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end

function y_hat = h_absolute(sv_pos, ref_pos, x)
    y_hat = zeros(size(sv_pos, 2), 1);
    
    for i = 1:size(sv_pos, 2)
        y_hat(i, 1) = norm(sv_pos(:, i) - x(1:3, 1)) + x(4);
    end
end

function H = compute_H_absolute(sv_pos, ref_pos, x)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: Measurement matrix (mx8)
    
    m = size(sv_pos, 2); % Number of satellites
    H = zeros(m, 4); % Initialize H matrix
   
    for i = 1:m
        H(i, 1:3) = (x(1:3, 1) - sv_pos(:, i))'/ norm(sv_pos(:, i) - x(1:3, 1));
        H(i, 4) = 1; % Clock bias term
    end
end


%% For Relative Positioning

function y_hat = h_relative(sv_pos, ref_pos, x)
    y_hat = zeros(size(sv_pos, 2), 1)
    
    for i = 1:size(sv_pos, 2)
        ref_los = (sv_pos(:, i) - ref_pos(1:3, 1)) / norm(sv_pos(:, i) - ref_pos(1:3, 1));
        y_hat(i, 1) = ref_los' * x(1:3,1);
    end
    
end

function H = compute_H_relative(sv_pos, ref_pos, x)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: Measurement matrix (mx8)
    
    m = size(sv_pos, 2); % Number of satellites
    H = zeros(m, 4); % Initialize H matrix
   
    for i = 1:m
        geometry_length = abs((sv_pos(:, i) - ref_pos(1:3, 1))') * x(1:3, 1) ./ norm(sv_pos(:, i) - ref_pos(1:3, 1));

        H(i, 1:3) = x(1:3, 1)'/ geometry_length;
        H(i, 4) = 0; % Clock bias term
    end
end