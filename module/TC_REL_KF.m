classdef TC_REL_KF
    properties
        state   % 현재 상태 추정
        covariance   % 현재 공분산 행렬
        A
    end
    
    methods
        % 생성자 메서드
        function obj = TC_REL_KF(initialState, initialCovariance)
            obj.state = initialState;
            obj.covariance = initialCovariance;
            obj.A = [];
            % obj.processNoise = processNoise;
            % obj.measurementNoise = measurementNoise;
            % obj.measurementMatrix = measurementMatrix;
            % obj.transitionMatrix = transitionMatrix;
        end

        function obj = update_A(obj, dt)
            % 3x3 단위 행렬
            I3 = eye(3);
            
            % 상태 전이 행렬 초기화 (16x16)
            F = zeros(8, 8);
            
            % 절대 위치와 속도에 대한 부분
            F(1:3, 1:3) = I3;            % 위치 -> 위치
            F(1:3, 4:6) = dt * I3;       % 속도 -> 위치
            F(4:6, 4:6) = I3;            % 속도 -> 속도
            
            % 절대 클럭 바이어스와 드리프트
            F(7, 7) = 1;                 % 클럭 바이어스 -> 클럭 바이어스
            F(7, 8) = dt;                % 클럭 바이어스 드리프트 -> 클럭 바이어스
            F(8, 8) = 1;                 % 클럭 바이어스 드리프트 -> 드리프트

            obj.A = F;
        end
        
        % Prediction 메서드
        function obj = predict(obj, Q, dt)

            obj = obj.update_A(dt)
            obj.state = obj.A * obj.state;
            % 공분산 예측
            obj.covariance = obj.A * obj.covariance * obj.A' + Q;
        end
        
        % Correction 메서드
        function obj = correct(obj, sv_pos, z_abs, z_rel, R)
            ref_pos = GNSS_LS(z_abs, length(z_abs), sv_pos);

            H = compute_H_relative(sv_pos, ref_pos, obj.state);
            z_hat = h_relative(sv_pos, ref_pos, obj.state);

            % 칼만 이득 계산
            K = obj.covariance * H' * inv(H * obj.covariance * H' + R);

            % 상태 업데이트
            obj.state = obj.state + K * (z_rel - z_hat);
            obj.covariance = (eye(size(obj.covariance)) - K * H) * obj.covariance;
        end
    end
end


function y_hat = h_relative(sv_pos, ref_pos, x)
    y_hat = zeros(size(sv_pos, 2), 1);
    
    for i = 1:size(sv_pos, 2)
        ref_los = (sv_pos(:, i) - ref_pos(1:3, 1)) / norm(sv_pos(:, i) - ref_pos(1:3, 1));
        y_hat(i, 1) = ref_los' * x(1:3,1) + x(7);
    end
end

function H = compute_H_relative(sv_pos, ref_pos, x)
    m = size(sv_pos, 2); % Number of satellites
    H = zeros(m, 8); % Initialize H matrix
   
    for i = 1:m
        H(i, 1:3) = (sv_pos(:, i) - ref_pos(1:3, 1)) / norm(sv_pos(:, i) - ref_pos(1:3, 1));
        H(i, 7) = 1; % Clock bias term
    end
end