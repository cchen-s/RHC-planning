
  A=eye(2);
  B=[0,0;0,0];
  u=[0;0];
  C=0.000001*eye(2);
  mu=[392;334];
  mu_w=[0;0];
  Sigma_w=[20,0;0,10];
  mu_v=[0;0];
  Sigma_v=[20,0.1;0.1,10];

  Sigma=[100,0.5;0.5,100];
  y=[323;222];

  [mu, Sigma] = kf_online(A, B, C, mu, mu_w, mu_v, Sigma, Sigma_w, Sigma_v,u,y);
 
  disp(mu)
  disp(Sigma);

  mu_2=mu;

function [mu_2, Sigma_2] = kf1(mu,Sigma)
  A=eye(2);
  B=[1,1];
  u=0;
  C=eye(2);
  %mu=[392;334];
  mu_w=[0;0];
  Sigma_w=[10,0;0,10];
  mu_v=[0;0];
  Sigma_v=[20,5;5,20];

  %Sigma=[100,0.5;0.5,100];

%   for i = 1:25
%       Sigma = kf_offline(A, B, C, mu, mu_w, mu_v, Sigma, Sigma_w, Sigma_v);
%   end
  %Sigma_2 = kf_offline(A, B, C, mu, mu_w, mu_v, Sigma, Sigma_w, Sigma_v);
  Sigma_2 = Sigma + Sigma_w;
  %disp(Sigma_2);

  mu_2=mu;
end



function [mu_2, Sigma_2] = kf_online(A, B, C, mu, mu_w, mu_v, Sigma, Sigma_w, Sigma_v, u, y)
    % prediction equations
    mu_1 = A * mu + B * u + mu_w;
    Sigma_1 = A * Sigma * A' + Sigma_w;

    % optimal kalman gain
    K = Sigma_1 * C' * inv(C * Sigma_1 * C' + Sigma_v);

    % update equations
    mu_2 = mu_1 + K * (y - C * mu_1 - mu_v);
    Sigma_2 = Sigma_1 - K * C * Sigma_1;
end

function Sigma_2 = kf_offline(A, B, C, mu, mu_w, mu_v, Sigma, Sigma_w, Sigma_v)
    % prediction equations
    Sigma_1 = A * Sigma * A' + Sigma_w;

    % optimal kalman gain
    K = Sigma * C' * inv(C * Sigma * C' + Sigma_v);

    % update equations
    Sigma_2 = Sigma_1 - K * C * Sigma_1;
end
