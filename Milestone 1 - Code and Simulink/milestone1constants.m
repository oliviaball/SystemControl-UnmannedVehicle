%constants that were provided, do not need to be changed 
%same constants for both the 5-state and 7-state models
%symbol= value; %units 
p= 0.037; %m 
Wr=0.18; %m 
Jeq=0.01802; %N-m-sec^2/rad
Beq=0.2963; %N-m-sec/rad
Keq=0.74; %n-m/A
Ke=2.2179e-3; %N-m/A
Kb=2.2179e-3; %vols-sec/rad
ra=7.4; %ohms 
la=0.002; %H
Nmotor=333.6375;

% step the parameters for the desired voltage input for right and left 
step_time=5; %seperate time for 0 - 5 seconds and 5-10 seconds

%variables for each wheel input, NEED TO BE CHANGED
u_r_i=10; %right wheel input initial(u1), from 0 to 5 seconds
u_r_f=9;  %right wheel input final(u1), from 5 to 10 seconds
u_l_i=5; %left wheel input initial(u2), from 0 to 5 seconds
u_l_f=3;  %left wheel input initial(u2), from 5 to 10 seconds

%initial state space values, set for x=0 but can change but 0 best
%each of these changes initial condition in integrator block
theta_i=0; %initial condition for theta
x_i=0; %initial condition for xi
y_i=0; %initial condition for yi
wr_i=0; %initial condition for angular velocity, right
wl_i=0; %intiial condition for angular velocity, left

%when implementing 7-state model 
ir_i=0.4; %winding current A, right
il_i=0.4; %winding current A, left

