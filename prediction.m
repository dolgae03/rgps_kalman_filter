function [x_pred, P_pred] = prediction(x, P, A, Q)
    % x: 이전 상태 추정값 (State Estimate)
    % P: 이전 공분산 추정값 (Covariance Estimate)
    % A: 상태 전이 행렬 (State Transition Matrix)
    % Q: 프로세스 노이즈 공분산 (Process Noise Covariance)
    
    % 상태 예측 (State Prediction)
    x_pred = A * x;
    
    % 공분산 예측 (Covariance Prediction)
    P_pred = A * P * A' + Q;
end