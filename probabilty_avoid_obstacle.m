% values_m = [84,112.601,148.293,180.065,214.482,247.779,279.24,301.772,334.758,359.535,381.737,417.89,441.401,465.5,490.706,521.90,557.08,578.978,591.084,587.548,561.817,533.55,496.112,467.74,440];
% values_n = [115,132.042,155.76,172.117,188.122,199.711,221.905,246.17,270.082,290.983,320.367,330.864,337.737,353.5,370.469,392.837,405.655,433.515,474.047,509.847,543.39,568.651,593.408,612.529,634];
% %disp(values_n);
% init_state=[56,95];
% 
% 
% 
% state0=[values_m, values_n];
% state0=transpose(state0);
% 
% [value, diff_array] = probabilty_avoid_obstacle1(state0);

function [value, diff_array] = probabilty_avoid_obstacle(t,values,mu_now,sigma_now)
tic
    len = 25-t;
    values_x=values(1:len);
    values_y=values(len+1:2*len);

    [value_part, diff_array_part] = probabilty_avoid_obstacle_part(t,values);

    [value_avoid_m, diff_array_avoid_m] = avoid_movable_obstacle(t,values_x,values_y,mu_now,sigma_now);
   
    disp(['value_avoid_m ',num2str(value_avoid_m)]);
  

    diff_array = zeros(25-t,2);
    for i = 1:25-t
       diff_array(i,1) = diff_array_part(i,1)*value_avoid_m + diff_array_avoid_m(i,1) * value_part;
       diff_array(i,2) = diff_array_part(i,2)*value_avoid_m + diff_array_avoid_m(i,2) * value_part;
    end
    value = value_part * value_avoid_m;

    value = -1e10*value;
    diff_array =  -1e10*diff_array;
    toc
    %disp(diff_array);
end



%the probability function of obstacle 1 
function [value, diff_array] = probabilty_avoid_obstacle_part(t,values)
    len = 25-t;
    values_x=values(1:len);
    values_y=values(len+1:2*len);

   [diff_array_avoid_ob, value_avoid_ob] = avoid_obstacle(t,values_x,values_y); 

   [diff_array_reach, value_reach]  = reach_lse(t,values_x,values_y); 

   diff_array = zeros(25-t,2);
   for i = 1:25-t
       diff_array(i,1) = diff_array_avoid_ob(i,1)*value_reach + diff_array_reach(i,1) * value_avoid_ob;
       diff_array(i,2) = diff_array_avoid_ob(i,2)*value_reach + diff_array_reach(i,2) * value_avoid_ob;
   end
   value = value_avoid_ob * value_reach;
    disp(['value_avoid_ob ',num2str(value_avoid_ob)]);
    disp(['value_reach ',num2str(value_reach)]);
%    value = -1e08*value;
%    diff_array =  -1e08*diff_array;
   %disp('diff_array');
   %disp(diff_array);
end



%移动障碍物的避障情况
function [prob_avoid_m,diff_array] = avoid_movable_obstacle(t,values_x,values_y,mu_now,sigma_now)
    variableContainer_m = cell(1, 25-t);
    variableContainer_diff_m = cell(1, 25-t);
    %mu=[301;255];
   % mu=[352;222];
    %mu=[301;246];
    %mu=[0;0];

   % Sigma=[100,0.5;0.5,100];
    mu=mu_now;
    Sigma=sigma_now;
    prob_avoid_m = 1;
    %创建避开移动障碍物的概率
    for i = 1:25-t
        % 创建变量名
        [mu_2, Sigma_2] = kf(mu,Sigma);
        mu=mu_2;
        Sigma=Sigma_2;

        dis=[values_x(i);values_y(i)] - mu;

        variableContainer_m{i} = 1-exp(-0.5*dis'*inv(Sigma)*dis);
        %该概率函数改自二维高斯分布的分布函数，所以很容易写出其导数
        variableContainer_diff_m{i} = exp(-0.5*dis'*inv(Sigma)*dis) * inv(Sigma)*dis;

        prob_avoid_m = prob_avoid_m * variableContainer_m{i};
    end
    diff_array = zeros(25-t,2);
    
    for i = 1:25-t
       diff_array(i,1) = prob_avoid_m/variableContainer_m{i} * variableContainer_diff_m{i}(1);
       diff_array(i,2) = prob_avoid_m/variableContainer_m{i} * variableContainer_diff_m{i}(2);
    end

end



function [diff_array, value_total] = avoid_obstacle(t,values_x,values_y)
   % syms x y;
    x=sym('x');
    y=sym('y');
    line1=141-x;
    line2=242-y;
    line3=x-219;
    line4=y-443;
    
    g1=line1+line2+(line1^2+line2^2)^(1/2);
    g2=line3+line4+(line3^2+line4^2)^(1/2);
    
    obstacle1=g1+g2+(g1^2+g2^2)^(1/2);
    
    ob1_prob=(exp(obstacle1)/(1+exp(obstacle1)));
    %ob1_prob=sigmoid(obstacle1);

    %the probability function of obstacle2
    line5=413-x;
    line6=169-y;
    line7=x-567;
    line8=y-232;
    
    g3=line5+line6+(line5^2+line6^2)^(1/2);
    g4=line7+line8+(line7^2+line8^2)^(1/2);
    
    obstacle2=g3+g4+(g3^2+g4^2)^(1/2);
    
    ob2_prob=(exp(obstacle2)/(1+exp(obstacle2)));
    %ob2_prob=sigmoid(obstacle2);

    %the probability function of obstacle 3
    line9=352-x;
    line10=448-y;
    line11=x-530;
    line12=y-560;
    
    g5=line9+line10+(line9^2+line10^2)^(1/2);
    g6=line11+line12+(line11^2+line12^2)^(1/2);
    
    obstacle3=g5+g6+(g5^2+g6^2)^(1/2);
    
    ob3_prob=(exp(obstacle3)/(1+exp(obstacle3)));
    %ob3_prob=sigmoid(obstacle3);
    ob_wall1_prob = (exp(x)/(1+exp(x)));
    ob_wall2_prob = (exp(y)/(1+exp(y)));
    ob_wall3_prob = (exp(768-x)/(1+exp(768-x)));
    ob_wall4_prob = (exp(706-y)/(1+exp(706-y)));

    ob_prob = ob1_prob*ob2_prob*ob3_prob*ob_wall1_prob*ob_wall2_prob*ob_wall3_prob*ob_wall4_prob ;
    
%     values_x=400;
%     values_y=360;
        
    %求对于x和y的微分
    diffx_ob_prob = diff(ob_prob,x);
    diffy_ob_prob = diff(ob_prob,y);

    save('avoid_obstacle_save.mat', 'diffx_ob_prob', 'diffy_ob_prob', 'ob_prob','x','y');
    

    
    values = zeros(1,25-t);
  
    parfor i = 1:25-t
        %ob_function1=ob_function;
        values(i) = double(subs(ob_prob, [x,y] , [values_x(i),values_y(i)]))
    end
  
    value_total = prod(values,'all');


    diff_array_x = zeros(25-t,1);
    diff_array_y = zeros(25-t,1);

    parfor i =1:25-t
        diff_array_x(i) = value_total/values(i) * double(subs(diffx_ob_prob, [x,y] , [values_x(i),values_y(i)]));
        diff_array_y(i) = value_total/values(i) * double(subs(diffy_ob_prob, [x,y] , [values_x(i),values_y(i)]));
    end
    diff_array=[diff_array_x, diff_array_y];
end

function [diff_array, value] = reach_lse(t,values_m,values_n)

%values_m,values_n是列向量
    t1=10;
    t2=15;
    
    % 创建符号变量数组
    
    mu_a = [360, 295];
    mu_b = [443, 630];
    
    Sigma_a = zeros(2, 2);
    Sigma_b = zeros(2, 2);
    

    
    m = sym('m',[25-t,1]);
    n = sym('n',[25-t,1]);
    
    variableContainer_b = cell(1, 25);
    variableContainer_a = cell(1, t1);
    variableContainer_f = cell(1, t1);
    
    %创建单独的到达B点和到达A点的概率

    for i = 1:t
        variableContainer_b{i} = 1;   
    end
    for i = t+1:t1+t2
        variableContainer_b{i} = exp(-((m(i-t)-mu_b(1))^2+(n(i-t)-mu_b(2))^2)/2000 + trace(Sigma_b) );   
    end
    
    for i = 1:t
        % 创建变量名
        variableContainer_a{i}= 1;
    end
    for i = t+1:t1
        % 创建变量名
        variableContainer_a{i} = exp(-((m(i-t)-mu_a(1))^2+(n(i-t)-mu_a(2))^2)/2000 + trace(Sigma_a) );
    end
    
    %根据STL的语义和迭代定义，计算整体的概率
    N=100;
    for i = 1:t1
       
        prob_temp=0;
        for j = 1:t2
            prob_temp = exp(N*variableContainer_b{i+j}) + prob_temp;
        end
        variableContainer_f{i} = variableContainer_a{i} * log(prob_temp)/N;
    end
    
    %计算最后F（A junction FB)的总概率
    
    prob_reach_temp = 0;
    for i = 1:t1
        prob_reach_temp = prob_reach_temp + exp(N*variableContainer_f{i});
    end
    prob_reach = log(prob_reach_temp)/N;
    %save('prob_reach.mat', 'prob_reach','m','n');
    %c=[m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15,m16,m17,m18,m19,m20,m21,m22,m23,m24,m25,n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20,n21,n22,n23,n24,n25];
    c=[m,n];
    prob_reach_function = matlabFunction(prob_reach,'Vars',{c});
    value = prob_reach_function([values_m,values_n]);

   
    
    diff_array_x=zeros(25-t,1);
    diff_array_y=zeros(25-t,1);
    parfor i=1:25-t
        diff_m = diff(prob_reach,m(i));
        diff_n = diff(prob_reach,n(i));

        diff_array_x(i)=double(subs(diff_m, [m;n] , [values_m;values_n]));
        diff_array_y(i)=double(subs(diff_n, [m;n] , [values_m;values_n]));

    end
    diff_array=[diff_array_x,diff_array_y];
end


function [diff_array, value] = reach(values_m,values_n)

%values_m,values_n是列向量
    t1=10;
    t2=15;
    
    % 创建符号变量数组
    
    mu_a = [443, 333];
    mu_b = [443, 630];
    
    Sigma_a = zeros(2, 2);
    Sigma_b = zeros(2, 2);
    
    
    
    m = sym('m',[25,1]);
    n = sym('n',[25,1]);
    
    variableContainer_b = cell(1, 25);

    variableContainer_a = cell(1, t1);
    variableContainer_f = cell(1, t1);
    
    %创建单独的到达B点和到达A点的概率
    for i = 1:t1+t2
        % 创建变量名
        %varName = strcat('prob_b', num2str(i));
        
        % 使用变量名进行操作
    %     eval([varName, ' = exp(-((m(i)-mu_b(1))^2+(n(i)-mu_b(2))^2)/20000 + trace(Sigma_b) )']);
    % 
    %     variableContainer_b{i} = eval(varName);
        variableContainer_b{i} = exp(-((m(i)-mu_b(1))^2+(n(i)-mu_b(2))^2)/2000 + trace(Sigma_b) );
        

    end
    
    for i = 1:t1
        % 创建变量名
        variableContainer_a{i}= exp(-((m(i)-mu_a(1))^2+(n(i)-mu_a(2))^2)/2000 + trace(Sigma_a) );
    end
    
    %根据STL的语义和迭代定义，计算整体的概率
    
    for i = 1:t1
        % 创建变量名
       % varName = strcat('prob_f', num2str(i));
        
        prob_temp=0;
        for j = 1:t2
            prob_temp = variableContainer_b{i+j}/(1- variableContainer_b{i+j}) + prob_temp;
        end
        % 使用变量名进行操作
    %     eval([varName, ' = variableContainer_a{i} * prob_temp/( 1 + prob_temp )']);
    % 
    %     variableContainer_f{i} = eval(varName);
    
        variableContainer_f{i} = variableContainer_a{i} * prob_temp/( 1 + prob_temp );
    end
    
    %计算最后F（A junction FB)的总概率
    
    prob_reach_temp = 0;
    for i = 1:t1
        prob_reach_temp = prob_reach_temp + variableContainer_f{i}/(1 - variableContainer_f{i});
        prob_reach = prob_reach_temp / (1+ prob_reach_temp);
    end
    
    save('prob_reach.mat', 'prob_reach','m','n');

    
    value = double(subs(prob_reach, [m;n] , [values_m;values_n]));


    diff_array=zeros(25,2);
    for i=1:25
        diff_m = diff(prob_reach,m(i));
        diff_n = diff(prob_reach,n(i));
    
        diff_m_value = double(subs(diff_m, [m;n] , [values_m;values_n]));
        diff_n_value = double(subs(diff_n, [m;n] , [values_m;values_n]));

        diff_array(i,1)=diff_m_value;
        diff_array(i,2)=diff_n_value;
    end
end





