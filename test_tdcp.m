num_samples = 100;

start_time = datetime(2024, 9, 8, 14, 30, 0);
stop_time = start_time + seconds(num_samples);

% 위성 시나리오 생성
sc = satelliteScenario(start_time, stop_time, num_samples, 'AutoSimulate', false);

satellite(sc, './data/galileo_tle.txt', "OrbitPropagator", "sgp4");

