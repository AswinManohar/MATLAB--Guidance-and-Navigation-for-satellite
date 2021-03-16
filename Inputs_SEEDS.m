%--------------------------------------------------------------------------    
% Cone section elements
%--------------------------------------------------------------------------    
d1   = 500;                                                                 % [m]   distance between target and chaser
r1   = 1;                                                                   % [m]   at point 1 (initial point) maximum height
r2   = 0.1;                                                                 % [m]   at point 2 (final point) maximum height
teta = atan((r1 - r2)/d1);                                                  % [rad] inclination angle

%--------------------------------------------------------------------------    
%   Orbital elements
%--------------------------------------------------------------------------
mu     = 3.986012*10^14;                                                    % [m^3/s^2]  Standard gravitational parameter
g0     = 9.81;                                                              % [m/s^2]    Zero gravity
r_E    = 6378.145*10^3;                                                     % [m]        Earth Radius
height = 400*10^3;                                                          % [m]        Reference altitude
r      = r_E + height;                                                      % [m]        Orbita Radius
omega  = (mu/r^3)^0.5;                                                      % [rad]      Target angular velocity
v      = sqrt(mu/r);                                                        % [m/s]      Tangential orbital speed

%------------------------------------------------------------------------
% Chaser
%--------------------------------------------------------------------------
lx   = 0.2;                     
ly   = 0.2;                     
lz   = 0.3405;  
m_c0 = 24; % initial mass [kg]                  
Jx0=(m_c0*(2*lx^2)/12);          
Jy0=(m_c0*(2*ly^2)/12);       
Jz0=(m_c0*(2*lz^2)/12);

J0 =[Jx0  0   0;                 % [kg*m^2]  Inertial tensor
     0  Jy0  0;               
     0   0  Jz0];
 
 
%--------------------------------------------------------------------------
%   Initial conditions – Hohmann Transfer
%--------------------------------------------------------------------------
% 
% Positions assigned in LVLH frame
% x0 = -10*10^3;
% y0 = 0;
% zc = 397*10^3; 						             % [m] Chaser orbit
% 
% zt      = 400*10^3;                                                        % [m]      Target orbit
% delta_z = zt - zc;                                                         % [m/s]    Difference of orbit between Chaser and Target 
% z0      = delta_z;
% Vx0  = v;                                                                % [m/s] Initial speed with no errors in ECI Frame with null Target speed 
% Vx0 = 3/2*omega*deltaz;						   % [m/s] Relative speed in LVLH when the Target and Chaser orbit is different
% 
%
% 
% x0_dot   = Vx0;                                                             % [m/s]    X speed 
% y0_dot   = 0;                                                               % [m/s]    Y speed 
% z0_dot   = 0;                                                               % [m/s]    Z speed
% 
% r      = r_E + height;                                                      % [m]      Orbita Radius
% omega_0  = (mu/r^3)^0.5;                                                    % [rad]    Chaser angular velocity
% omega0   = [0,-omega_0,0];                                                  % [rad/s]  Chaser angular velocity in the 
%                                                                             %          local orbital frame
% eul_ang0 = [0,0,0];                                                 	      % [rad]    Euler angles between the orbital 
%                                                                             %          reference frame and the local orbital frame
%                                                                             %          (chaser) -- only if aligned
% Pos0     = [x0 y0 z0]';
% Vel0     = [x0_dot y0_dot z0_dot]';

%--------------------------------------------------------------------------
%   Initial conditions – Radial Boost
%--------------------------------------------------------------------------
Vx0  = v;                                                                   % [m/s]     Initial speed
 
x0       = -3*10^3;                                                         % [m]      X position 
y0       = 0;                                                               % [m]      Y position 
z0       = 0;                                                               % [m]      Z position
x0_dot   = Vx0;                                                             % [m/s]    X speed 
y0_dot   = 0;                                                               % [m/s]    Y speed 
z0_dot   = 0;                                                               % [m/s]    Z speed
omega0   = [0,0,0];                                                         % [rad/s]  Chaser angular velocity in the 
                                                                            %          local orbital frame
eul_ang0 = [0,0,0];                                                         % [rad]    Euler angles between the orbital 
                                                                            %          reference frame and the local orbital frame
                                                                            %          (chaser) -- only if aligned
Pos0     = [x0 y0 z0]';
Vel0     = [x0_dot y0_dot z0_dot]';


% %--------------------------------------------------------------------------
% %   Initial conditions – Cone maneuver
% %--------------------------------------------------------------------------
% 
% Vmin = 0.01;                                                                % [m/s] Minimum speed of thruster, Vx slide 51
% Vx0  = 0.15;                                                                % [m/s] Initial speed - first impulse slide 51, Maneuver
%  
% x0       = -500;                                                               % [m]      X position 
% y0       = 0;                                                               % [m]      Y position 
% z0       = 0;                                                               % [m]      Z position
% x0_dot   = Vx0;                                                             % [m/s]    X speed 
% y0_dot   = 0;                                                               % [m/s]    Y speed 
% z0_dot   = 0;                                                               % [m/s]    Z speed
% omega0   = [0,0,0];                                                    % [rad/s]  Chaser angular velocity in the 
%                                                                             %          local orbital frame
% %                                                           
% eul_ang0 = [0,0,0];                                                  % [rad]    Euler angles between the orbital 
%                                                                             %          reference frame and the local orbital frame
%                                                                             %          (chaser) - only if aligned
% Pos0     = [x0 y0 z0]';
% Vel0     = [x0_dot y0_dot z0_dot]';

%--------------------------------------------------------------------------
% Final point
%--------------------------------------------------------------------------
% xf       = -4;                   % [m]  
% yf       = 0;                    % [m]
% zf       = 0;                    % [m]

%--------------------------------------------------------------------------
% Simulation parameters
%--------------------------------------------------------------------------
tsp_0  = 0.02;                                                              % [s]   Zero shoot time
Dt     = tsp_0;                                                             % [s]   Time step
tfin   = 10000;                                                             % [s]   Maximum time of simulation
cont   = 0;                                                                 % [s]   Index
t      = 0:Dt:tfin;                                                         % [s]   Vector of time

%%  Section 2: Thrusts and Moments due to the thrusters
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%   Default thruster specs.
%--------------------------------------------------------------------------

Isp1  = 220;                                                                % [s]     Specific impulse
c1    = Isp1*g0;                                                            % [m/s]   Dumping speed
Tmax  = 1;                                                                 % [N]     Maximum thrust
n     = 2;                                                                 % [--]    Number of thrusters (1X+3X) or (3Z+4Z)
F     = n*Tmax;

%--------------------------------------------------------------------------
%   Default RW specs.
%--------------------------------------------------------------------------
%------------------------------------------------------------------------
% Reaction Wheels 
%--------------------------------------------------------------------------
gmax        = 0.1;                                                         % [N*m]       RW Maximum torque
Is          = 0.02;                                                        % [kg*m^2]    RW Moment of inertia
om_0        = [0,0,0]';
tau_RW      = 1;                                                           % [s]         RW Time constant
p_RW        = 0.6;                                                         % [rad/s]     RW Pole
Kr          = 1;



%% Section 3: Errors due to external disturbances 
% Constant and random errors are considered for the forces.
% Constant error is considered for the moments.
%--------------------------------------------------------------------------
%   Thrust errors due to external disturbances
%--------------------------------------------------------------------------

F_err     = [0.00001,0.00001,0.00001];                                      % [N]  Thrust constant error 

Sample_time_F = 1  ;                                                        % [s]  Sampling time for the thrust 
                                                                            %      error 
%--------------------------------------------------------------------------
%   Constant error for the moment due to external disturbances
%--------------------------------------------------------------------------

M_ext = [0.00001,0.00001,0.00001];                                                   % [N*m]  Moment constant error
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%   Thrust due to air drag model
%--------------------------------------------------------------------------

rho = 1*10^(-12);                                                           % [kg/m^3] Air density obtained as medium value for 500 km orbit Fehse
Vx  = omega*r;                                                              % [m/s]    Orbital velocity
CD  = 2.2;                                                                  % [--]     Drag coefficient for satellite of compact shapes
A   = ly*lz;                                                                % [m^2]    Cross section
F_D = -1/2*rho*CD*A*Vx^2;                                                   % [N]      Force due to the drag
F_D = [F_D 0 0];                                                           % [N]      Force vector


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data for simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dt      = 0.01;                                                             % [s] Sample time
Dt_guid = 1;                                                                % [s] Update frequency of the guidance algorithm
% Attitude angles as constant vector -- TO BE DELETED if the attitude
% control is included.
att_const = [0.01;0.01;0.01]';
