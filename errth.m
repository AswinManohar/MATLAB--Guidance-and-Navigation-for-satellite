function T_err =errth(u)

T = u(1:3);

Tmax = 1;
n=2;
% Random error -- As input
erth = normrnd(0,0.05,1,1);
    
% Constant error: 1% of Tmax
T_mag = erth + 0.01*n*Tmax + T;

% Directional Error

T_err = (normrnd(1,0.05,1,1))*u;