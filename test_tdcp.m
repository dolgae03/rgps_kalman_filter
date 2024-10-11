% MATLAB 스크립트: genFactorTDCP 함수 테스트
% 
% -2359913.784193 14502572.250594 21835005.400214 
% -2363062.194739 14502332.174669 21834814.379466 
% -1892.491000 1000.000000
% -3119984.708229 4086870.275167 3761602.538938 
% -3119977.082511 4086856.628033 3761594.760359 

clear

% 상수 정의
c = 299792458;           % 빛의 속도 (m/s)
omega = 7.292115E-5;     % 지구의 회전율 (rad/s)

% 사용자 상태 정의
% x(:,1): 현재 상태, x(:,2): 다음 상태
% 상태 벡터는 [x, y, z, clock_bias_GPS, clock_bias_GALILEO, clock_bias_Beidou]로 구성됩니다.

% 이전 사용자 상태 (원점에 위치)
x_cur = [-3.11998e+06,4.08687e+06,3.7616e+06, -1.38378e-09 * c, 0, 0, 0]';

% 현재 사용자 상태 (x 축으로 10m 이동, 작은 시계 바이어스 변화)
x_next = [-3.11998e+06,4.08686e+06,3.76159e+06, -9.20897e-09 * c, 0, 0, 0]';

% 상태 행렬 생성
x = [x_cur, x_next];  % 6 x 2 행렬

% 위성 위치 정의 (ECEF 좌표계, 단위: m)
sv_pos_cur_all = [-2359913.784193,14502572.250594,21835005.400214]';       % 현재 시점의 위성 위치
sv_pos_next_all = [-2363062.194739,14502332.174669,21834814.379466]';   % 다음 시점의 위성 위치 (y 축으로 약간 이동)

% 위성 수
num_sv = 1;

% 위성 유형 정의 (0: GPS L1C)
constl_type = [1];  % 1 x num_sv 벡터

% GPS L1 주파수 및 파장 계산
L1_frequency = 1575.42e6;  % Hz
lambda = c / L1_frequency; % 파장 (m)

% 캐리어 위상 측정값 생성
% 초기 캐리어 위상 (사이클 단위)
ph1 = 0;  % 사이클

% 사용자와 위성 간의 거리 계산
r_cur = norm(sv_pos_cur_all - x_cur(1:3));
r_next = norm(sv_pos_next_all - x_next(1:3));

delta_r = -1892.49;


% 캐리어 위상 변화 (사이클 단위)
delta_ph = lambda*delta_r;

% 다음 시점의 캐리어 위상 계산
ph2 = ph1 + delta_ph;

% 캐리어 위상 측정값 행렬 생성 (num_sv x 2)
ph = [ph1, ph2];

% genFactorTDCP 함수 실행
factor = genFactorTDCP(x, ph, sv_pos_cur_all, sv_pos_next_all, constl_type);
factor = calculate_tdcp()

% 결과 출력
disp(['Factor (Residual): ', num2str(factor * 1000)]);
