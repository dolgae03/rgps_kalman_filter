function [x_est, covariance] = GNSS_LS(pr, N, sv_pos)
%GNSS_LS_position_velocity - Calculates position, velocity, clock offset, 
%and clock drift using unweighted iterated least squares. Separate
%calculations are implemented for position and clock offset and for
%velocity and clock drift
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%   no_GNSS_meas          Number of satellites for which measurements are
%                         supplied
%   predicted_r_ea_e      prior predicted ECEF user position (m)
%   predicted_v_ea_e      prior predicted ECEF user velocity (m/s)
%
% Outputs:
%   est_r_ea_e            estimated ECEF user position (m)
%   est_v_ea_e            estimated ECEF user velocity (m/s)
%   est_clock             estimated receiver clock offset (m) and drift (m/s)
 
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega = 7.292115E-5;  % Earth rotation rate in rad/s

% Begins

% POSITION AND CLOCK OFFSET

sv_pos = sv_pos';

% Setup predicted state
% x_pred(1:3,1) = x_init;
x_pred(1:3, 1) = [0; 0; 0];
x_pred(4,1) = 0;
err = 1;

% Repeat until convergence
while err > 5e-7
    
    % Loop measurements
    for j = 1:N

        % Predict approx range 
        delta_r = sv_pos(j,:)' - x_pred(1:3);
        approx_range = sqrt(delta_r' * delta_r);

        % Calculate frame rotation during signal transit time using (8.36)
        C = [1, omega * approx_range / c, 0;...
             -omega * approx_range / c, 1, 0;...
             0, 0, 1];

        % Predict pseudo-range using (9.143)
        delta_r = C *  sv_pos(j,:)' - x_pred(1:3);
        range = sqrt(delta_r' * delta_r);
        pr_pred(j,1) = range + x_pred(4);
        
        % Predict line of sight and deploy in measurement matrix, (9.144)
        G(j,1:3) = - delta_r' / range;
        G(j,4) = 1;
        
    end % for j
        
    % Unweighted least-squares solution, (9.35)/(9.141)
    x_est = x_pred + inv(G' *G) * G' *(pr - pr_pred(1:N));

    % Test convergence    
    err = sqrt((x_est - x_pred)' * (x_est - x_pred));
    
    % Set predictions to estimates for next iteration
    x_pred = x_est;

end

% Covariance matrix calculation
covariance = inv(G' * G);
end