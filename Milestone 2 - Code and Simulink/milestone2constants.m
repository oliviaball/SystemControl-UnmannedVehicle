%% Constants to control the system- Run in Sections (before simulink)
%constants that were provided, do not need to be changed 
%symbol= value; %units 
r= 0.037; %m, wheel radius 
Wr=0.18; %m 
Jeq=0.01802; %N-m-sec^2/rad
Beq=0.2963; %N-m-sec/rad
Keq=0.74; %n-m/A
Ke=2.2179e-3; %N-m/A
Kb=2.2179e-3; %vols-sec/rad
ra=7.4; %ohms 
la=0.002; %H
M=1.59; %kg
g= 9.81; %m/s^2

%input voltages for right and left wheels - step parameters
step_time=10; %seperate time for 0 - 5 seconds and 5-10 seconds
u_r_i=5; %right wheel input initial(u1), from 0 to 5 seconds
u_r_f=3;  %right wheel input final(u1), from 5 to 10 seconds
u_l_i=10; %left wheel input initial(u2), from 0 to 5 seconds
u_l_f=9;  %left wheel input initial(u2), from 5 to 10 seconds

%initial state space values
theta_i=0; %initial condition for theta
x_i=0; %initial condition for xi
y_i=0; %initial condition for yi
wr_i=0; %initial condition for angular velocity, right
wl_i=0; %intiial condition for angular velocity, left
%create a matrix with all of these initial conditions
initial_cond=[x_i, y_i, theta_i, wr_i, wl_i];

%noise constant, when set to zero these apply no noise
v_i=0; %set to a value between 0 and 0.1523
v_w=0; %set to a value between 0 and 0.01148 (0.01148 highest values for error less than 1)
v_txy=0; %set to a value between 0 and 1

%disturbance constants- keep all 0 for no disurbancs
mu=0; %friction coefficent, described road coefficents
Fd=0; % kg, any additional load added to the model
slope=0; %degrees, slope of the road in degrees
threshold=5; %time when disturance is applied, change to change when disturbance occurs
% do not change these - used in distruance blocks as constants
alpha=(slope*pi)/180; % rad, convert slope into radians 
Mg=M*g;
Mgmu=M*g*mu;

%% Linearize the matrix models- (before simulink)
% Create the matrix, t=theta,wr=angular velocity right, wl=angular velocity left
% Define the nominal point xn=[x y theta wr wl]
x=0; %position in x direction
y=0; %position in y direction
t=0; %theta 
wr=0; %angular velocity right, max 3.349
wl=0; %angular velocity left, max 3.349
%create the nominal point=xn=[x y theta wr wl]
xn=[x y t wr wl];

%create the A matrix
a1=[0 0 -(r*sin(t)*(wr+wl))/2 (r*cos(t))/2 (r*cos(t))/2];
a2= [0 0 (r*cos(t)*(wr+wl))/2 (r*sin(t))/2 (r*sin(t))/2];
a3= [ 0 0 0 r/Wr -r/Wr];
a4= [ 0 0 0 -(Beq+(Ke*Kb)/ra)/Jeq 0];
a5= [ 0 0 0 0 -(Beq+(Ke*Kb)/ra)/Jeq];
%Linearized model parameters 
A=[a1; a2; a3; a4; a5]
B=[0 0; 0 0; 0 0; Keq/(ra*Jeq) 0; 0 Keq/(ra*Jeq)]
C=[0 0 0 r/2 r/2];
D=zeros(1,2);

%% Determine the controllability and observablity- (before or after)
%Make the p matrix
p_1=B
p_2=A*B
p_3=A^2*B
p_4=A^3*B
p_5=A^4*B
P=[p_1 p_2 p_3 p_4 p_5]
P_rank=rank(P)

%make the Q matrix 
q_1=C;
q_2=C*A;
q_3=C*A^2;
q_4=C*A^3;
q_5=C*A^4;
Q=[q_1; q_2; q_3; q_4; q_5]
Q_rank=rank(Q)

%get normal form matricies  
%determine eigenvalues (D) using the A matrix
[eigen_vector,eigen_value]=eig(A)
M=eigen_vector;
Minv=inv(M)
Bn=Minv*B
Cn=C*M

%% Simulation Used to Find Max Noise - uncomment for monte carlo analysis
% N=40; % number of simulations
% range=40; %how many points are desired
% step=linspace(0,0.016,range);
% 
% for i=1:N
%     
%     v_w=0; %set as initial condition
%     v_w= v_w + step(1,i); %add a tolerance value (step) to previous loop 
%     
%     %implement the model in simulink for given v_ value
%     simOut(i) = sim('fivestatespace_revised',... 
%                 'SimulationMode','normal',...
%                 'StopTime','10');
%             
% end

%% Find Out Max Noise - Tolerated - uncomment for monte carlo analysis
% e=zeros(1,N);%define a matrix for error at each value to be plugged into
% step_val=step(1,2)-step(1,1); %find the step value for variance 
% 
% for i=1:N
%     
%     getinput=simOut(i).get('logsout'); %pull data from simulation i
%     v_lin_model=getinput.get('v_lin').Values.Data; %pull linearized output
%     v_nonlin_model=getinput.get('v_nonlin').Values.Data; %pull nonlinearized output
%     %compare effects of noise to linearized model model
%     error=abs(v_nonlin_model-v_lin_model); %calculate error at each variance
%     max_error=max(error); %calculate max error for simulation
%     e(:,i)=max_error*100; %find error percentage
% 
% end
% 
% %set to value of tolerance we will accept in the model 
% tol=1; 
% error_allowed=find((e<=tol));
% max_noise_input=step(length(error_allowed))
% stem(step,e)
% title('Max Error at Various Variance Values When Noise Applied to the System Speeds (w_r and w_l)' )
% xlabel('Variance Input (Step 0.00041)')
% ylabel('Error Between Model with Noise and Without Noise')

%% Plot Output Noise - Plot Output and Error- (Run After Simulink)
s_in=speed_nonlin.data;
s_i=speed_lin.data;
t_s=speed_lin.time;
e_s=abs(s_in-s_i)*100;

figure (1)
subplot(2,1,1)
plot(t_s,s_in)
hold on 
plot(t_s,s_i)
hold off
ylabel('Output')
xlabel('Time (Seconds)')
title('UV Starts Climbing Hill with Slope 20 degrees and Experiences Input Noise with Variance=0.1523') 
legend('Nonlinear','Linearized')
subplot(2,1,2)
plot(t_s,e_s)
ylabel('Difference (Percentage)')
xlabel('Time (Seconds)')
title('Error Between Model with Noise and Model without Noise')

