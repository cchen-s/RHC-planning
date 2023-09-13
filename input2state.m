
function state_list = input2state(angular_velocity_list,velocity_list)
    init_state=[56,95,atan2(20,28)];
    state=init_state';
    T=1;
    state_list = zeros(26,3);
    state_list(1,:) = init_state;
    for i =1:25
       state_next = comp_next_state(state,angular_velocity_list(i),velocity_list(i),T);
       state_list(i+1,:) = state_next';
       state = state_next;
    end
%     disp(state_list)
end

function state_next = comp_next_state(state,angular_velocity,velocity,T)
    B=[T*cos(state(3)),0; T*sin(state(3)),0; 0,T];
    state_next = state + B*[velocity; angular_velocity];

end