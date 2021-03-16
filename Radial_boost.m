clear all;
%--------------------------------------------------------------------------
%                               DATA INPUT
%--------------------------------------------------------------------------
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

% 
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
J0_inv = [1/Jx0 0 0;
           0  1/Jy0 0;
           0    0  1/Jz0];
%--------------------------------------------------------------------------
%   Default thruster specs.
%--------------------------------------------------------------------------

Isp1  = 220;                                                                % [s]     Specific impulse
c1    = Isp1*g0;                                                            % [m/s]   Dumping speed
Tmax  = 1;                                                                 % [N]     Maximum thrust
n     = 2;                                                                 % [--]    Number of thrusters (1X+3X) or (3Z+4Z)
F     = n*Tmax;

%--------------------------------------------------------------------------
%   Thrust errors due to external disturbances
%--------------------------------------------------------------------------

F_err     = [0.00001,0.00001,0.00001];                                      % [N]  Thrust constant error 

Sample_time_F = 1  ;                                                        % [s]  Sampling time for the thrust 
                                                                            %      error 
%------------------------------------------------------------------------
% Reaction Wheels 
%--------------------------------------------------------------------------
gmax        = 0.1;                                                         % [N*m]       RW Maximum torque
Is          = 0.02;                                                        % [kg*m^2]    RW Moment of inertia
om_0        = [0,0,0]';
tau_RW      = 1;                                                           % [s]         RW Time constant
p_RW        = 0.6;                                                         % [rad/s]     RW Pole
Kr          = 1;

%--------------------------------------------------------------------------
% Moments due to external disturbances
%--------------------------------------------------------------------------
M_ext = [0.00001,0.00001,0.00001];
%--------------------------------------------------------------------------
% Simulation parameters
%--------------------------------------------------------------------------
tsp_0  = 0.01;                                                              % [s]   Zero shoot time
Dt     = tsp_0;                                                             % [s]   Time step
tfin   =10000;                                                             % [s]   Maximum time of simulation
cont   = 0;                                                                 % [s]   Index
t      = 0:Dt:tfin; % [s]   Vector of time 
%--------------------------------------------------------------------------
%   Thrust due to air drag model
%--------------------------------------------------------------------------

rho = 1*10^(-12);                                                           % [kg/m^3] Air density obtained as medium value for 500 km orbit Fehse
Vx  = omega*r;                                                              % [m/s]    Orbital velocity
CD  = 2.2;                                                                  % [--]     Drag coefficient for satellite of compact shapes
A   = ly*lz;                                                                % [m^2]    Cross section
F_D = -1/2*rho*CD*A*Vx^2;                                                   % [N]      Force due to the drag
F_D = [F_D 0 0];                                                           % [N]      Force vector


%-------------------------------------------------------------------------
%                           TRAJECTORY
%------------------------------------------------------------------------
% RB period data. Final position is not defined in the inputs file
xf = -0.5e3;
yf = 0;
zf= 0;
Posf = [xf,yf,zf];
delta_x = Posf(1)-Pos0(1);
%Chaser data ----- Input file

Fx = 0;
Fy = 0; %We leave it like this cuz we dont fucking care

%Definition of impulse time depending on the maximum thrust

delta_vz = (omega/4)*delta_x; %delta V burn mangitude for the manouver
impulse_time = delta_vz*m_c0 / F; %calculated with maximum

%We calculate the maneuver total time (Period).
gamma_z = (omega^2/(4*pi))*delta_x;
total_maneuver_time = 0.5*(omega*delta_x)/(2 * gamma_z);
second_burn_time = total_maneuver_time - impulse_time;
%--------------------------------------------------------------------------
%Application of thruster magnitude and shoot corrections (NOT NEEDED)
%--------------------------------------------------------------------------
T_th1 = [Fx Fy Tmax];
T_th2 = [Fx Fy Tmax];
T_th1_mag = errth(T_th1);
T_th2_mag = errth(T_th2);
T_th1_bs = [T_th1_mag' [0 0 0]'];
T_th2_bs = [T_th2_mag' [0 0 0]'];
T_th1 = erth_shoot(T_th1_bs,normrnd(0,0.05,1,1));
T_th2 = erth_shoot(T_th2_bs,normrnd(0,0.05,1,1));
%--------------------------------------------------------------------------
%Momentum created by thrusters (NOT NEEDED)
%--------------------------------------------------------------------------
d = lz/2; %m. Distance from thruster to CoM. Rotation in Y
M_th_abs = (T_th2 - T_th1)*d; %Momentum created by thrusters. Created only two times
M_th = [0;M_th_abs(3);0]; %Thruster momentum vector


%--------------------------------------------------------------------------       
%                               ATTITUDE       
%--------------------------------------------------------------------------
%INITIAL CONDITIONS
%--------------------------------------------------------------------------
omega0   = [1,1,1];          % [rad/s]  Chaser angular velocity in the local orbital frame        

% %                                                           
eul_ang0 = [0.5,0.5,0.5];  % [rad]    Euler angles between the orbital 
q_initial = eul2quat(eul_ang0);

%-------------------------------------------------------------------------
% CONTROLLER DESIGN
%-------------------------------------------------------------------------
eul_ang_lin = [0,0,0];
q0_lin = eul2quat(eul_ang_lin)'; %Angles for linearization of system
q0_lin = [1 0 0 0]'; %In quaternions, just in case
omega_lin = [0 0 0] + [0.000000001 0.000000001 0.000000001]; %With zeros the lqr function gives errors

A = [0 0 0 0.5 0 0;
     0 0 0 0 0.5 0;
     0 0 0 0 0 0.5;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
 
 B = [0 0 0;
      0 0 0;
      0 0 0;
      1/Jx0 0 0;
      0 1/Jy0 0;
      0 0 1/Jz0];
  
  Q = 5*eye(6);
  R = 1/gmax^2*eye(3);
  
 K_lqr =lqr(A,B,Q,R);
 eig(A-B*K_lqr)
 
%--------------------------------------------------------------------------
% SIMULATION RUN
%--------------------------------------------------------------------------

results = sim('RB_model');

%--------------------------------------------------------------------------
% Extraction of results
%--------------------------------------------------------------------------
%Trajectory

%Ideal

x = results.pos_x;
y = results.pos_y;
z = results.pos_z;   
x_dot = results.xdot;
y_dot = results.ydot;
z_dot = results.zdot;

%Real with thruster errors
x2 = results.pos_x2;
y2 = results.pos_y2;
z2 = results.pos_z2; 
x_dot2 = results.xdot2;
y_dot2 = results.ydot2;
z_dot2 = results.zdot2;

%Real with external disturbances
x_err = results.pos_xerr;
y_err = results.pos_yerr;
z_err = results.pos_zerr; 
x_dot_err = results.xdoterr;
y_dot2_err = results.ydoterr;
z_dot2_err = results.zdoterr;

%Attitude
q = results.q;
q_error = results.q_error;
[psi, theta, phi] = quat2angle(q);
omega_b = results.omega_b;

%--------------------------------------------------------------------------
%                               POST-PROCESSING
%--------------------------------------------------------------------------
close all
%--------------------------------------------------------------------------
% Trajectory
%--------------------------------------------------------------------------

% Trajectory (LVLH frame)
figure
plot(x,z)
hold on
plot(x2,z2)
hold on
plot(x_err,z_err)
xlabel ('V-BAR (m)')
ylabel ('R-BAR (m)')
title('Trajectory (LVLH frame)')
grid on
legend ('Ideal','Real th', 'Real ext');

figure
plot(t,x-x_err)
xlabel ('time (s))')
ylabel ('position error (m)')
title('Position error')
grid on

% Absolute position (ideal)
% figure
% plot(t,x)
% xlabel ('time (s))')
% ylabel ('position (m)')
% title('Position X')
% grid on
% legend

% Velocity (ideal)
% figure
% plot(t,x_dot)
% hold on
% plot(t,y_dot)
% hold on
% plot(t,z_dot)
% xlabel ('time (s))')
% ylabel ('speed (m/s)')
% title('Spacecraft velocity')
% grid on
% legend ('Vx','Vy','Vz')

%--------------------------------------------------------------------------
% Attitude
%--------------------------------------------------------------------------

%Angles

% figure
% plot(t,psi)
% title('Angle psi')
% grid on
% xlabel('Time(s)')
% ylabel('Angle (rad)')
% xlim([0 100])
% 
% figure
% plot(t,theta,'r')
% title('angle theta')
% grid on
% xlabel('Time(s)')
% ylabel('Angle (rad)')
% xlim([0 100])
% 
% figure
% plot(t,phi,'k')
% title('angle phi')
% grid on
% xlabel('Time(s)')
% ylabel('Angle (rad)')
% xlim([0 100])

figure
plot(t,psi)
hold on
plot(t,theta,'r')
hold on
plot(t,phi,'k')
title('angles')
grid on
xlabel('Time(s)')
ylabel('Angle (rad)')
legend ('Psi','Theta','Phi')
xlim([0 100])

%Controller signal
% figure
% plot(t,results.M_cont)
% title('Controller signal')
% grid on
% xlabel('Time(s)')
% ylabel('Torque (Nm)')
% legend ('Mrw_x','Mrw_y','Mrw_z')
% xlim([0 100])
% ylim([-0.15 0.15])

%Angular speed
figure
plot(t,omega_b)
title('angular speeds')
grid on
xlabel('Time(s)')
ylabel('Angular speed (rad/s)')
legend ('wb_x','wb_y','wb_z')
xlim([0 100])
