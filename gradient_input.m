function [value, diff_array_input] = gradient_input(input_list,t,mu_now, sigma_now)
   % t=7;    
    load('input_x.mat','input_x');
    velocity_list = input_list(1:25-t);
    angular_velocity_list = input_list(25-t+1:50-2*t);
    
    velocity_list1 = [input_x(1:t);velocity_list];
    angular_velocity_list1 = [input_x(26:t+25);angular_velocity_list];

    state_list = input2state(angular_velocity_list1,velocity_list1);
    values = [state_list(t+2:26,1);state_list(t+2:26,2)];
    
    state_list=state_list(t+1:26,:);

%     velocity_list = input_list(1:25);
%     angular_velocity_list = input_list(26:50);
% 
%     state_list = input2state(angular_velocity_list,velocity_list);
%     values = [state_list(2:26,1);state_list(2:26,2)];

    [value, diff_array] = probabilty_avoid_obstacle(t,values,mu_now,sigma_now);
%     disp('diff_array');
%     disp(diff_array);

    diff_array_input = zeros(25-t,2);
    
    T=1;

    matrix = ones(25-t, 25-t);
    matrix = triu(matrix, 0);
    matrix_sum = matrix * diff_array;
%     disp('matrix_sum');
%     disp(matrix_sum);

    for i =1:25-t
        diff_array_input(i,1) = matrix_sum(i,1)*T*cos(state_list(i,3)) + matrix_sum(i,2)*T*sin(state_list(i,3));
        diff_omega=0;
        for n = i+1 : 25-t
            diff_x2theta=0;
            diff_y2theta=0;
            for j = i+1 : n
               diff_x2theta = diff_x2theta - T*T*velocity_list(j) * sin(state_list(j,3));
               diff_y2theta = diff_y2theta + T*T*velocity_list(j) * cos(state_list(j,3));
            end
            diff_omega = diff_omega + diff_array(n,1) * diff_x2theta + diff_array(n,2) * diff_y2theta;
        end
        diff_array_input(i,2) = diff_omega;
         
    end

end