%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Error on the thrust magnitude
%           file created by elisa.capello@polito.it, 22/04/2013
%
%  Input: nominal thrust, maximum thrust
%  
%  NB This function is called for each thruster considered in the model on
%     the Simulink file. 
%     
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function Tmag =erth_mag(u,erth)

T = u(1:3);
Tmag = zeros(3,1);

% Global variables
Tmax = 25;

% Random error -- As input
% erth = normrnd(0,0.05,1,1);
    
% Constant error: 1% of Tmax
Tmag = erth + 0.01*Tmax + T;


