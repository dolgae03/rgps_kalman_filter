n = size(sv_pos, 1);
norm_diff = zeros(n, 1);

for i = 1:n
    % 두 위성 벡터의 차이
    diff_vec = sv_pos{i, 2} - sv_pos{i, 1};
    
    % 차이 벡터의 노름 계산
    norm_diff(i) = norm(diff_vec);
end
save_norm = sv_pos{end, 1} - sv_pos{end, 2};

% 노름 값의 플롯 (2D Plot)
figure(32);
plot(1:n, norm_diff, '-o', 'MarkerFaceColor', 'b');
xlabel('Index');
ylabel('Norm of Difference');
title('Norm of Difference between Satellite Vectors');
grid on;
