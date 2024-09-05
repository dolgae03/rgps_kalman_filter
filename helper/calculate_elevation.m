function elevation = calculate_elevation(user_pos, sv_pos)
    % user_pos: 사용자의 위치 (1x3 벡터) [x_user, y_user, z_user]
    % sv_pos: 위성의 위치 (1x3 벡터) [x_sv, y_sv, z_sv]
    
    % 벡터 계산 (위성으로의 벡터)
    vector = sv_pos - user_pos;

    % 거리 계산 (위성과의 직선 거리)
    range = sqrt(vector(1)^2 + vector(2)^2 + vector(3)^2);

    % Azimuth 계산 (단위: 라디안)
    azimuth = atan2(vector(2), vector(1));  % y축, x축 성분 사용

    % Elevation 계산 (단위: 라디안)
    elevation = asin(vector(3) / range);  % z축 성분과 range 사용
end
