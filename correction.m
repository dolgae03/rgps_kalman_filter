function [x_corr, P_corr] = correction(x_pred, P_pred, z, R)
    % x_pred: 예측된 상태 추정값 (Predicted State Estimate)
    % P_pred: 예측된 공분산 추정값 (Predicted Covariance Estimate)
    % z: 실제 측정값 (Measurement)
    % H: 측정 행렬 (Measurement Matrix)
    % R: 측정 노이즈 공분산 (Measurement Noise Covariance)
    
    H = compute_H_relative_to_A(x_pred, z);

    % 칼만 이득 계산 (Kalman Gain)
    K = P_pred * H' / (H * P_pred * H' + R);
    

    z = z(:, 1) - z(:, 2);
    % 상태 수정 (State Correction)
    x_corr = x_pred + K * (z - H * x_pred);
    
    % 공분산 수정 (Covariance Correction)
    P_corr = (eye(size(P_pred)) - K * H) * P_pred;
end

function H = compute_H_relative_to_A(x, z)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % Returns H: Measurement matrix (mx8)
    
    m = size(z, 1); % Number of satellites
    H = zeros(m, 8); % Initialize H matrix
   
    for i = 1:m
        H(i, 1:3) = x(1:3) / (z(i, 1) - z(i, 2));
        H(i, 7) = 1; % Clock bias term
    end
end
