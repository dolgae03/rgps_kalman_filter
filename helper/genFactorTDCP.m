function factor = genFactorTDCP(x, ph, sv_pos_cur_all, sv_pos_next_all, constl_type)

c = 299792458;           % Speed of light in m/s
omega = 7.292115E-5;     % Earth rotation rate in rad/s

x_cur = x(:,1);
x_next = x(:,2);

num_state = size(x,1);
num_sv = size(ph,1);
d_motion = zeros(num_sv,1); 
G = zeros(num_sv, num_state);

    for k = 1:num_sv

        sv_pos_cur = sv_pos_cur_all(:,k);
        sv_pos_next = sv_pos_next_all(:,k);
        approx_r =  sv_pos_cur - x_cur(1:3,1);
        approx_range = sqrt(approx_r' * approx_r);
        C = [1, omega * approx_range / c, 0;...
             -omega * approx_range / c, 1, 0;...
             0, 0, 1];

        r_cur = C*sv_pos_cur - x_cur(1:3,1);
        range_cur = sqrt(r_cur' * r_cur); 
        los_cur = r_cur' / range_cur;
        
        G(k,1:3) = -los_cur ;
        G(k,4:num_state) = 0;
        G(k,constl_type(k)+3)=1;

%         clk_bias = x_cur(constl_type(k)+3);


        r_next = C*sv_pos_next - x_next(1:3);
        range_next = sqrt(r_next' * r_next); 
        los_next = r_next' / range_next;

        D = los_next*sv_pos_next - los_cur*sv_pos_cur; % satellite motion
        g = los_next*x_cur(1:3)- los_cur*x_cur(1:3); % change in LOS
        d_motion(k) =  D-g;
 
      
    end

    tdcp_pred = G*(x_next-x_cur);
    tdcp = (ph(:,2) - ph(:,1)) - d_motion;

    factor = (tdcp_pred-tdcp)'*(tdcp_pred-tdcp);
%     if factor > 500
%         disp("TDCP")
%     end
%     
    
    

 
end