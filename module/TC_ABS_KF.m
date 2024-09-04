classdef TC_ABS_KF
    properties
        state   % 현재 상태 추정
        covariance   % 현재 공분산 행렬
        A
    end
    
    methods
        function obj = TC_ABS_KF(initialState, initialCovariance)
            obj.state = initialState;
            obj.covariance = initialCovariance;
            obj.A = [];
        end

        function obj = update_A(obj, dt)
            % 3x3 단위 행렬
            I3 = eye(3);
            
            F = zeros(8, 8);
            
            F(1:3, 1:3) = I3;            % 위치 -> 위치
            F(1:3, 4:6) = dt * I3;       % 속도 -> 위치
            F(4:6, 4:6) = I3;            % 속도 -> 속도
            
            F(7, 7) = 1;                 % 클럭 바이어스 -> 클럭 바이어스
            F(7, 8) = dt;                % 클럭 바이어스 드리프트 -> 클럭 바이어스
            F(8, 8) = 1;                 % 클럭 바이어스 드리프트 -> 드리프트
            
            obj.A = F;
        end
        
        % Prediction 메서드
        function obj = predict(obj, Q, dt)
            obj = obj.update_A(dt)
            obj.state = obj.A * obj.state;

            obj.covariance = obj.A * obj.covariance * obj.A' + Q;
        end
        
        % Correction 메서드
        function obj = correct(obj, sv_pos, z_abs, R)
            H = compute_H_absolute(sv_pos, obj.state);
            
            y = h_absolute(sv_pos, obj.state);

            z_hat = [y];
            z = [z_abs];

            K = obj.covariance * H' * inv(H * obj.covariance * H' + R);

            obj.state = obj.state + K * (z - z_hat);
            obj.covariance = (eye(size(obj.covariance)) - K * H) * obj.covariance;
        end
    end
end


function y_hat = h_absolute(sv_pos, x)
    y_hat = zeros(size(sv_pos, 2), 1);
    
    for i = 1:size(sv_pos, 2)
        y_hat(i, 1) = norm(sv_pos(:, i) - x(1:3, 1)) + x(7);
    end
end

function H = compute_H_absolute(sv_pos, x)
    m = size(sv_pos, 2); % Number of satellites
    H = zeros(m, 8); % Initialize H matrix
   
    for i = 1:m
        H(i, 1:3) = (x(1:3, 1) - sv_pos(:, i))'/ norm(sv_pos(:, i) - x(1:3, 1));
        H(i, 7) = 1; % Clock bias term
    end
end