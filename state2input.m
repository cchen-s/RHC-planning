
function [angular_velocity_list,velocity_list] = state2input(values_m, values_n)

%     values_m = [84,112.601,148.293,180.065,214.482,247.779,279.24,301.772,334.758,359.535,381.737,417.89,441.401,451.349,490.706,521.90,557.08,578.978,591.084,587.548,561.817,533.55,496.112,467.74,440];
%     values_n = [115,132.042,155.76,172.117,188.122,199.711,221.905,246.17,270.082,290.983,320.367,330.864,337.737,368.793,370.469,392.837,405.655,433.515,474.047,509.847,543.39,568.651,593.408,612.529,634];
    %disp(values_n);
    init_state=[56,95];
    values_m=[init_state(1),values_m];
    values_n=[init_state(2),values_n];
    angular_velocity_list=zeros(25,1);
    velocity_list=zeros(25,1);
    T=1;
    for i = 1:24
        [angular_velocity_list(i),velocity_list(i)] = calculate_velocity(values_m(i), values_n(i), values_m(i+1), values_n(i+1), values_m(i+2), values_n(i+2),T);
    end 
    velocity_list(25) = sqrt((values_m(26)-values_m(25))^2 + (values_n(26)-values_n(25))^2) / T;
    
%     disp(velocity_list);
%     disp(angular_velocity_list);

end

% % 创建图像
% figure(1);
% axis([0 768 0 706]);
% line([0,0],[706,0]);
% line([0,768],[0,0]);
% line([0,768],[706,706]);
% line([768,768],[0,706]);
% rectangle('Position',[141 242 78 201],'edgecolor','k','facecolor','g','linewidth',1.8) 
% axis equal
% rectangle('Position',[342 448 188 112],'edgecolor','k','facecolor','g','linewidth',1.8) 
% axis equal
% rectangle('Position',[413 169 154 63],'edgecolor','k','facecolor','g','linewidth',1.8) 
% axis equal
% 
% hold on;  % 保持绘图区域
% plot(values_m(1:26), values_n(1:26), 'yo', 'MarkerSize', 2, 'MarkerFaceColor', 'red');
% hold on;

function [angular_velocity, velocity] = calculate_velocity(x_a, y_a, x_b, y_b, x_c, y_c, T)
    % Calculate the angle between points A and B
    angle_a_b = atan2(y_b - y_a, x_b - x_a);
    
    % Calculate the angle between points B and C
    angle_b_c = atan2(y_c - y_b, x_c - x_b);
    
    % Calculate the difference in angles
    angle_difference = angle_b_c - angle_a_b;
    
    % Ensure the angle difference is between -pi and pi
    angle_difference = mod(angle_difference + pi, 2 * pi) - pi;
    
    % Calculate the angular velocity needed to reach the desired angle in time T
    angular_velocity = angle_difference / T;
    velocity = sqrt((y_b-y_a)^2 + (x_b-x_a)^2) / T;
end
