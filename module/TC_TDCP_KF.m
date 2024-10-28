classdef TC_TDCP_KF
    properties
        prev_state
        state   % 현재 상태 추정
        covariance   % 현재 공분산 행렬
        A
        u
        use_external_force
        cov_log
        inital_log
    end
    
    methods
        % 생성자 메서드
        function obj = TC_TDCP_KF(initialState, initialCovariance, use_external_force)
            obj.prev_state = initialState;
            obj.state = initialState;
            obj.covariance = initialCovariance;
            obj.A = [];
            obj.u = [];
            obj.use_external_force = use_external_force;
            obj.inital_log = [];
            obj.cov_log = [];
        end

        function obj = update_A(obj, dt)
            I3 = eye(3);
            
            F = zeros(16, 16);
            
            % 절대 위치와 속도에 대한 부분
            F(1:3, 1:3) = I3;            % 위치 -> 위치
            F(1:3, 4:6) = dt * I3;       % 속도 -> 위치
            F(4:6, 4:6) = I3;            % 속도 -> 속도
            
            % 절대 클럭 바이어스와 드리프트
            F(7, 7) = 1;                 % 클럭 바이어스 -> 클럭 바이어스
            F(7, 8) = dt;                % 클럭 바이어스 드리프트 -> 클럭 바이어스
            F(8, 8) = 1;                 % 클럭 바이어스 드리프트 -> 드리프트
            
            % 상대 위치와 속도에 대한 부분
            F(9:11, 9:11) = I3;          % 상대 위치 -> 상대 위치
            F(9:11, 12:14) = dt * I3;    % 상대 속도 -> 상대 위치
            F(12:14, 12:14) = I3;        % 상대 속도 -> 상대 속도
            
            % 상대 클럭 바이어스와 드리프트
            F(15, 15) = 1;               % 상대 클럭 바이어스 -> 상대 클럭 바이어스
            F(15, 16) = dt;              % 상대 클럭 바이어스 드리프트 -> 상대 클럭 바이어스
            F(16, 16) = 1;               % 상대 클럭 바이어스 드리프트 -> 드리프트

            obj.A = F;
        end
        
        function obj = update_u(obj, dt)
            a_1 = Gravity_ECEF(obj.state(1:3, 1));
            a_2 = Gravity_ECEF(obj.state(1:3, 1) + obj.state(9:11, 1));

            u_temp = zeros(16, 1);

            u_temp(1:3, 1) = 1/2 * a_1 * dt^2;
            u_temp(4:6, 1) = a_1 * dt;

            u_temp(9:11, 1) = 1/2 * (a_2 - a_1) * dt^2;
            u_temp(12:14) = (a_2 - a_1) * dt;

            obj.u = u_temp;
        end


        % Prediction 메서드
        function obj = predict(obj, Q, dt)
            obj = obj.update_A(dt);
            obj = obj.update_u(dt);
            
            obj.state = obj.A * obj.state;
            if obj.use_external_force
                obj.state = obj.state + obj.u;
            end

            % 공분산 예측
            obj.covariance = obj.A * obj.covariance * obj.A' + Q;
        end
        
        % Correction 메서드
        function obj = inital_correct(obj, sv_pos1, sv_pos2, z_abs, z_rel, z_range, R)
            H_abs = compute_H_absolute(sv_pos1, obj.state, length(z_abs));
            H_rel = compute_H_relative(sv_pos1, sv_pos2, obj.state, length(z_rel));
            H_range = compute_H_range(sv_pos1, obj.state, length(z_range));

            H = [H_abs; H_rel; H_range;];

            y_abs = h_absolute(sv_pos1, obj.state, length(z_abs));
            y_rel = h_relative(sv_pos1, sv_pos2, obj.state, length(z_rel));
            y_range = h_range(sv_pos1, obj.state, length(z_range));

            z_hat = [y_abs; y_rel; y_range;];
            z = [z_abs; z_rel; z_range;];

            % 칼만 이득 계산
            K = obj.covariance * H' * inv(H * obj.covariance * H' + R);

            % 상태 업데이트
            obj.prev_state = obj.state;

            obj.state = obj.state + K * (z - z_hat);
            obj.covariance = (eye(size(obj.covariance)) - K * H) * obj.covariance;

            obj.inital_log(:, end+1) = obj.state;
            obj.cov_log(:,:, end+1) = obj.covariance;
        end
    
           
        function obj = final_correct(obj, z_pos, R)
            H = compute_H_pos(obj.state, length(z_pos));
            z_hat = h_pos(obj.state, length(z_pos));
            z = z_pos;

            % 칼만 이득 계산
            K = obj.covariance * H' * inv(H * obj.covariance * H' + R);

            % 상태 업데이트
            obj.prev_state = obj.state;

            obj.state = obj.state + K * (z - z_hat);
            obj.covariance = (eye(size(obj.covariance)) - K * H) * obj.covariance;
            
            obj.final_log(:, end+1) = obj.state;
        end

        function [pos, cov] = get_relative_pos_info(obj)
            % Extract position and orientation information from the state
            pos = [obj.state(1:3) + obj.state(9:11); obj.state(7) + obj.state(15)];
            
            % Define indices for state vectors
            pos_indices_1 = 1:3;    % First part of the position
            pos_indices_2 = 9:11;   % Second part of the position
            orient_indices_1 = 7;   % First part of the orientation
            orient_indices_2 = 15;  % Second part of the orientation
            
            % Covariance for the position part
            cov_pos_1 = obj.covariance(pos_indices_1, pos_indices_1);
            cov_pos_2 = obj.covariance(pos_indices_2, pos_indices_2);
        
            cov_1_1 = cov_pos_1 + cov_pos_2;
            
            cov_pos_1 = obj.covariance(orient_indices_1, pos_indices_1);
            cov_pos_2 = obj.covariance(orient_indices_2, pos_indices_2);
            cov_1_2 = cov_pos_1 + cov_pos_2;
        
            cov_orient_1 = obj.covariance(orient_indices_1, orient_indices_1);
            cov_orient_2 = obj.covariance(orient_indices_2, orient_indices_2);
        
            cov_2_2 = cov_orient_1 + cov_orient_2;

            cov_pos_1 = obj.covariance(pos_indices_1, orient_indices_1);
            cov_pos_2 = obj.covariance(pos_indices_2, orient_indices_2);
            cov_2_1 = cov_pos_1 + cov_pos_2;
        
            cov = [cov_1_1, cov_2_1;
                   cov_1_2, cov_2_2];
        end
    end
end


function y_hat = h_absolute(sv_pos, x, num)
    y_hat = zeros(num, 1);
    
    for i = 1:size(sv_pos, 2)
        y_hat(i, 1) = norm(sv_pos(:, i) - x(1:3, 1)) + x(7);
    end
end

function H = compute_H_absolute(sv_pos, x, num)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: Measurement matrix (mx8)
    
    H = zeros(num, 16); % Initialize H matrix
   
    for i = 1:num
        H(i, 1:3) = (x(1:3, 1) - sv_pos(:, i))'/ norm(sv_pos(:, i) - x(1:3, 1));
        H(i, 7) = 1; % Clock bias term
    end
end

function y_hat = h_relative(sv_pos1, sv_pos2, x, num)
    y_hat = zeros(num, 1);
    
    for i = 1:num
        sv_1_to_sat_norm = norm(sv_pos1(:, i) - x(1:3, 1));
        sv_2_to_sat_norm = norm(sv_pos2(:, i) - (x(1:3, 1) + x(9:11, 1)));

        y_hat(i, 1) = sv_1_to_sat_norm - sv_2_to_sat_norm + x(15);
    end
end

function H = compute_H_relative(sv_pos1, sv_pos2, x, num)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: Measurement matrix (mx8)
    
    H = zeros(num, 16); % Initialize H matrix
   
    for i = 1:num
        sv_1_to_sat = sv_pos1(:, i) - x(1:3, 1);
        sv_2_to_sat = sv_pos2(:, i) - (x(1:3, 1) + x(9:11, 1));

        sv_1_to_sat_norm = norm(sv_1_to_sat);
        sv_2_to_sat_norm = norm(sv_2_to_sat);

        H(i, 1:3) = - sv_1_to_sat' / sv_1_to_sat_norm + sv_2_to_sat' / sv_2_to_sat_norm;
        H(i, 9:11) = + sv_2_to_sat' / sv_2_to_sat_norm;
        H(i, 15) = 1; % Clock bias term
    end
end

function y_hat = h_range(sv_pos, x, num)
    y_hat = zeros(num, 1);
    
    for i = 1:num
        y_hat(i, 1) = norm(x(9:11, 1));
    end
end

function H = compute_H_range(sv_pos, x, num)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: M
    H = zeros(num, 16); % Initialize H matrix
   
    for i = 1:num
        H(i, 9:11) =  x(9:11, 1) / norm(x(9:11, 1));
    end
end

function y_hat = h_pos(x, num)
    y_hat = zeros(num, 1);
    y_hat(1:3,1) = x(1:3, 1);
end

function H = compute_H_pos(x, num)
    % x: State vector (8x1) [Delta x; Delta y; Delta z; Delta vx; Delta vy; Delta vz; b; b_dot]
    % z: Matrix of pesudorange with respect to Satellite A (mx2)
    % sv_pos: Matrix of sv pos with in ecef frame (mx3)
    % Returns H: M
    H = zeros(num, 16); % Initialize H matrix
    
    H(1:3, 1:3) = eye(3);
end