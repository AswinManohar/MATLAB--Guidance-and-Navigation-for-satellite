%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Error on the shoot direction
%           file created by elisa.capello@polito.it
%
%  Input: thrust including magnitude errors
%  
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Ter_sh =erth_shoot(u, erth)

%--------------------------------------------------------------------------
% Input from the previous block
%--------------------------------------------------------------------------

T  = u(1:3,1);
%T = u(1:3); Added by me

Ter_sh = zeros(3,1);   
    if T(1) ~= 0    
    
%--------------------------------------------------------------------------   
% Constant error
%--------------------------------------------------------------------------

    erangd  = [1,1,1]; 
    erang   = erangd.*pi/180;
    Th      = T;
    Ttot    = sqrt(Th(1)^2+Th(2)^2+Th(3)^2);
    delta   = acos(Th(1)/Ttot);
    eps     = acos(Th(2)/Ttot);
    zeta    = acos(Th(3)/Ttot);
    Thang   = [delta,eps,zeta];
    ang     = erang + Thang;
    Terang  = cos(ang);

%--------------------------------------------------------------------------   
% Random error
%--------------------------------------------------------------------------   

    erangd  = erth; 
    erang   = erangd.*pi/180;
    Th_n    = Terang;
    Ttot    = sqrt(Th_n(1)^2 + Th_n(2)^2 + Th_n(3)^2);
    delta   = acos(Th_n(1)/Ttot);
    eps     = acos(Th_n(2)/Ttot);
    zeta    = acos(Th_n(3)/Ttot);
    Thang   = [delta,eps,zeta];
    ang     = erang + Thang;
    Terangt = cos(ang);

    Ter = T + Terangt';
    
    else 
        
    Ter = T;
    
    end

Ter_sh(1:3,1) = Ter(1:3,1);

