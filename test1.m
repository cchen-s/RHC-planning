
% values = [ 1.07669081e+02,1.14000000e+02;
%        1.31451961e+02,1.13727583e+02;
%        1.56676638e+02,1.13820793e+02;
%        1.78570560e+02,1.13512640e+02;
%        2.07676257e+02,1.12575818e+02;
%        2.32762564e+02,1.11442190e+02;
%        2.61684880e+02,1.09916138e+02;
%        2.89556433e+02,1.07918996e+02;
%        3.04622788e+02,1.06788147e+02;
%        3.25459754e+02,1.05472842e+02;
%        3.54161629e+02,1.04079868e+02;
%        3.70551863e+02,1.03250038e+02;
%        3.93379893e+02,1.01957773e+02;
%        4.11027014e+02,1.01061778e+02;
%        4.30550138e+02,1.00146457e+02;
%        4.50620174e+02,9.90701949e+01;
%        4.73339399e+02,9.72840144e+01;
%        4.93382094e+02,9.52373452e+01;
%        5.11153062e+02,9.38398521e+01;
%        5.27798518e+02,9.27170967e+01;
%        5.43128119e+02,9.14612264e+01;
%        5.60308254e+02,9.04241927e+01;
%        5.82466804e+02,8.95046108e+01;
%        6.10813870e+02,8.76577366e+01;
%        6.34886499e+02,8.66096855e+01 ];
% 
% values_m = transpose(values(:, 1));
% values_n = transpose(values(:, 2));

% values_m = [84,112.601,148.293,180.065,214.482,247.779,279.24,301.772,334.758,359.535,381.737,417.89,441.401,451.349,490.706,521.90,557.08,578.978,591.084,587.548,561.817,533.55,496.112,467.74,440];
% values_n = [115,132.042,155.76,172.117,188.122,199.711,221.905,246.17,270.082,290.983,320.367,330.864,337.737,368.793,370.469,392.837,405.655,433.515,474.047,509.847,543.39,568.651,593.408,612.529,634];
% %disp(values_n);
init_time=clock;
values_m = [84,112,148,180,214,247,279,301,334,359,381,417.89,441.401,465.5,490.706,521.90,557.08,578.978,591.084,587.548,561.817,533.55,496.112,467.74,440];
values_n = [115,132,155,172,188,199,221,246,270,290,320,330.864,337.737,353.5,370.469,392.837,405.655,433.515,474.047,509.847,543.39,568.651,593.408,612.529,634];
%disp(values_n);
init_state=[56,95];
target=[360,295;443,630];


state0=[values_m, values_n];
state0=transpose(state0);

values_x=state0(1:25);
values_y=state0(26:50);

%对于状态量进行优化的结果
%options = optimset('GradObj', 'on','Display','iter', 'MaxIter', 40);
threshold = -8.0e09;
%options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
%options = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton','HessianApproximation','lbfgs','SpecifyObjectiveGradient',true,'ObjectiveLimit',threshold,'MaxIterations',300);
% % options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton','MaxIterations',20)
% input_list0=[velocity_list_1;angular_velocity_list_1];
% state_list = input2state(angular_velocity_list_1,velocity_list_1);
% state0 = [state_list(2:26,1);state_list(2:26,2)];
% 

% 
% [x,fval,exitflag,output] = fminunc(@(t)(probabilty_avoid_obstacle(t)), state0, options);
% disp(x);
% disp(fval);
% disp(exitflag);
% disp(output);



%对于输入速度和角速度的梯度计算，并且以此进行优化
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','HessianApproximation','lbfgs','SpecifyObjectiveGradient',true,'ObjectiveLimit',threshold,'MaxIterations',300,'OptimalityTolerance', 1e-4);
% [angular_velocity_list,velocity_list] = state2input(values_m, values_n);
% input_list0=[velocity_list;angular_velocity_list];
% fun = @gradient_input;
% velocity_lb=ones(1,25)*20;
% velocity_ub=ones(1,25)*50;
% angular_velocity_lb=-0.7*ones(1,25);
% angular_velocity_ub=0.7*ones(1,25);
% lb=[velocity_lb,angular_velocity_lb];
% ub=[velocity_ub,angular_velocity_ub];
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% nonlcon=[];
% 
% 
% [input_x,input_fval,input_exitflag,input_output] = fmincon(fun, input_list0, A,b, Aeq, beq,lb,ub,nonlcon,options);
% save('input_x.mat', 'input_x');
% 
% state_list_final = input2state(input_x(26:50),input_x(1:25));
% 
% disp(input_x);
% disp(input_fval);
% disp(input_exitflag);
% disp(input_output);


%在t时刻进行的 mpc
index_n=0;
fail_num=0;
while index_n<50
    index_n=index_n+1;
   
    load('input_x.mat','input_x');
    t=0;
    r_ob=5;
    R_robot=70;
    state_ob=[0;0];
    state_ob(1)=rand(1)*50+325;
    state_ob(2)=rand(1)*50+200;
    
    mu_initial = [301;255];
    sigma_initial = [100,20;20,80];
    Sigma_v=[50,25;25,50];
    Sigma_w=[10,0;0,10];
    
    mu_now=mu_initial;
    Sigma_now=sigma_initial;
    
    while t<25
        options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','HessianApproximation','lbfgs','SpecifyObjectiveGradient',true,'ObjectiveLimit',threshold,'MaxIterations',300,'OptimalityTolerance', 1e-6); 
        %t=7;
        input_list0=[input_x(t+1:25);input_x(t+26:50)];
        fun = @(x)gradient_input(x,t,mu_now, Sigma_now);
        velocity_lb=ones(1,25-t)*20;
        velocity_ub=ones(1,25-t)*50;
        angular_velocity_lb=-0.7*ones(1,25-t);
        angular_velocity_ub=0.7*ones(1,25-t);
        lb=[velocity_lb,angular_velocity_lb];
        ub=[velocity_ub,angular_velocity_ub];
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        nonlcon=[];
        
        
        [input_x_t6,input_fval,input_exitflag,input_output] = fmincon(fun, input_list0, A,b, Aeq, beq,lb,ub,nonlcon,options);
        %save('input_x_t6.mat', 'input_x');
        velocity_list = input_x_t6(1:25-t);
        angular_velocity_list = input_x_t6(25-t+1:50-2*t);
        
        velocity_list1 = [input_x(1:t);velocity_list];
        angular_velocity_list1 = [input_x(26:t+25);angular_velocity_list];
        input_x=[velocity_list1;angular_velocity_list1];
        
        state_list_final = input2state(angular_velocity_list1,velocity_list1);
    
        t=t+1;
        
        while (state_list_final(t,1) - state_ob(1))^2+(state_list_final(t,2) - state_ob(2))^2>=R_robot^2
            t=t+1;
            if t>26
                break
            end
            Sigma_now = Sigma_now + Sigma_w;
        end
        position_observed = mvnrnd(state_ob,Sigma_v,1);
        position_observed = position_observed';
    %     disp(mu_now);
    %     disp(Sigma_now);
        [mu_2, Sigma_2] = kf_online(mu_now, Sigma_now, Sigma_w, Sigma_v, position_observed);
        mu_now=mu_2;
        Sigma_now=Sigma_2;
    %     disp(input_x);
    %     disp(input_fval);
    %     disp(input_exitflag);
    %     disp(input_output);
    %     disp(mu_now);
    %     disp(Sigma_now);
    %     disp(position_observed);
        disp(['t ',num2str(t)]);
    end
    
    avoid_flag = true;
    for i=1:26
       if (state_list_final(i,1)-state_ob(1))^2+ (state_list_final(i,2)-state_ob(2))^2 <=r_ob^2
          avoid_flag = false;
          fail_num=fail_num+1;
          break
       end
    end


end


% 创建图像
figure(1);
axis([0 768 0 706]);
plot(target(:,1),target(:,2),'yo', 'MarkerSize', 5, 'MarkerFaceColor', 'blue')
line([0,0],[706,0]);
line([0,768],[0,0]);
line([0,768],[706,706]);
line([768,768],[0,706]);
rectangle('Position',[141 242 78 201],'edgecolor','k','facecolor','g','linewidth',1.8) 
axis equal
rectangle('Position',[342 448 188 112],'edgecolor','k','facecolor','g','linewidth',1.8) 
axis equal
rectangle('Position',[413 169 154 63],'edgecolor','k','facecolor','g','linewidth',1.8) 
axis equal

hold on;  % 保持绘图区域
%plot(x(1:25), x(26:50), 'yo', 'MarkerSize', 5, 'MarkerFaceColor', 'red');
plot(state_list_final(1:26,1), state_list_final(1:26,2), 'yo', 'MarkerSize', 2, 'MarkerFaceColor', 'red');
%plot(values_m, values_n, 'yo', 'MarkerSize', 2, 'MarkerFaceColor', 'green');
hold on;

%mu=[323 222];
mu=position_observed';
Sigma=[100,0.5;0.5,100];
%p = mvncdf([0 0],[2 3],mu,Sigma);

x1 = mu(1)-30:.2:mu(1)+30;
x2 = mu(2)-30:.2:mu(2)+30;
[X1,X2] = meshgrid(x1,x2);
X = [X1(:) X2(:)];
y = mvnpdf(X,mu,Sigma)*2*pi*sqrt(det(Sigma));
y = reshape(y,length(x2),length(x1));

contour(x1,x2,y,[0.1  0.9])
xlabel('x')
ylabel('y')
hold on;
end_time=clock;
etime(end_time,init_time)
%line([0 0 1 1 0],[1 0 0 1 1],'Linestyle','--','Color','k')

function [mu_2, Sigma_2] = kf_online(mu, Sigma, Sigma_w, Sigma_v, y)
    % prediction equations
  A=eye(2);
  B=[0,0;0,0];
  u=[0;0];
  C=eye(2);
  %mu=[392;334];
  mu_w=[0;0];
  mu_v=[0;0];

    mu_1 = A * mu + B * u + mu_w;
    Sigma_1 = A * Sigma * A' + Sigma_w;

    % optimal kalman gain
    K = Sigma_1 * C' * inv(C * Sigma_1 * C' + Sigma_v);

    % update equations
    mu_2 = mu_1 + K * (y - C * mu_1 - mu_v);
    Sigma_2 = Sigma_1 - K * C * Sigma_1;
end