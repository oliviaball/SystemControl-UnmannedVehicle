%% Constants to control the system- Run in Sections (before simulink)
format long 
%Constants that were provided, do not need to be changed,symbol= value; %units 
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
fullrank = 5; % Change depending on full rank
%Change depending on the case you are using below - changes titles 
position='Initial 3';
noise='Disturbance';

%%%%%%%%%%%%%%%%%% Uncomment the case that you are interested in %%%%%%%%%%%%%%%%%%
% --------------- Case 1  -------------------------
% p_i=[0 0 deg2rad(0)]; %initial position  (degress)
% p_d=[10 20 deg2rad(0)]; %desired position (degress)
% pos_i=[0 0 0]; %initial position for the graphs
% pos_d=[10 20 0]; %desired position for the graphs

% --------------- Case 2 ---------------------------
% p_i=[0 0 deg2rad(115)]; %initial position (degrees)
% p_d=[10 15 deg2rad(35)]; %desired position (degrees)
% pos_i=[0 0 115]; %initial position for the graphs
% pos_d=[10 15 35]; %desired position for the graphs

% --------------- Case 3 ---------------------------
p_i=[0 0 deg2rad(250)]; %initial position (degrees)
p_d=[10 18 deg2rad(100)]; %desired position (degrees)
pos_i=[0 0 250]; %initial position for the graphs
pos_d=[10 18 100]; %desired position for the graphs

%Translation of goal back to origin
gto1 = [cos(p_d(3)) sin(p_d(3)) -(p_d(1)*cos(p_d(3))+p_d(2)*sin(p_d(3)))] ; 
gto2 = [-sin(p_d(3)) cos(p_d(3)) -(p_d(2)*cos(p_d(3))-p_d(1)*sin(p_d(3)))];
gto3 = [0 0 1];
gto=[ gto1; gto2; gto3];   
 
%Put in terms of initial and desired x and y
or_i = gto*[p_i(1);p_i(2);1];
or_d = gto*[p_d(1);p_d(2);1];

%Define delta x, delta y and theta for initial conditions
dx=or_d(1)-or_i(1); %difference in x (desired- initial)
dy=or_d(2)-or_i(2); %difference in y (desired -initial)
theta=p_i(3)-p_d(3); %difference in theta (initial-desired)

%Define the initial conditions
p_initial=sqrt(dx^2+dy^2); %rho
alpha_initial=-theta+atan2(dy,dx); %alpha
beta_initial=-theta-alpha_initial; %beta
wr_initial=0; %angular velocity right
wl_initial=0; %angular velocity left

%Define initial condition matrix for integrator blocks 
x_initial=[p_initial alpha_initial beta_initial 3.3275 3.3275]; 

%Define nominal point
pos=1; %initial rho- very small 
alpha=0; % initial alpha
beta=0; %initial beta
w_r= 3.3275; %max angular velocity
w_l=3.3275; %max angular velocity
A=[0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0];
%Define each row of linearized A matrix 
A(1,2)=(r*sin(alpha)*(w_r+w_l))/2;
A(1,4)=(-r*cos(alpha)/2);
A(1,5)=(-r*cos(alpha)/2);
A(2,1)=(-r*sin(alpha)*(w_r+w_l))/(2*pos^2);
A(2,2)=(r*cos(alpha)*(w_r+w_l))/(2*pos);
A(2,4)=(r*sin(alpha)/(2*pos))-(r/Wr);
A(2,5)=(r*sin(alpha)/(2*pos))+(r/Wr);
A(3,1)=(r*sin(alpha)*(w_r+w_l))/(2*pos^2);
A(3,2)=(-r*cos(alpha)*(w_r+w_l))/(2*pos);
A(3,4)=(-r*sin(alpha)/(2*pos));
A(3,5)=(-r*sin(alpha)/(2*pos));
A(4,4)=(-Beq+((Keq*Kb)/ra))/Jeq;
A(5,5)=(-Beq+((Keq*Kb)/ra))/Jeq;
B=[0 0; 0 0; 0 0; Keq/(ra*Jeq) 0; 0 Keq/(ra*Jeq)]; %B matrix
C= [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0]; %Output Matrix
D=zeros(3,2); %Same rows as C and columns as B for run
yn=zeros(1,3); % Define extra input set to zero, same rows as D
un=[0 0]; %Define extra input, set to zero

%Make sure that the system is completely controllable - rank 5
P=ctrb(A,B);
rankp=rank(P);
if (rankp == fullrank)
    fprintf ('The system is completely controllable\n')
else 
    fprintf ('The system is NOT completely controllable\n')
end

%Make sure that the system is completely observable - rank 5
Q=obsv(A,C);
rankq = rank(Q);
if (rankq == fullrank)
    fprintf ('The system is completely observable\n')
else 
    fprintf ('The system is NOT completely observable\n')
end

if (rankq == fullrank && rankp == fullrank)
    fprintf ('Continue\n')
else 
    fprintf ('Achieve Full Rank Before Moving On\n')
end

%Decide where to place controller poles- will take trial and error done in
%monte carlo
p=[-7.5 -0.4 -0.1 -0.0375 -0.65]; %[-16 -17 -1.2 -0.19 -0.017];
k=place(A,B,p);

%Design the observer poles and place them 
p_o=[-41 -36 -43 -44 -45]; %[-48 -54 -29 -30 -38];
bc_1=place(A',C',p_o);
bc=bc_1';
ac=A-bc*C;
k_z=B-bc*D;

%% Define noises and disturbances if occur 
%noise constant, when set to zero these apply no noise
v_i=0; %variance to input, cannot be large
v_o=0; %variance output, cannot be large

%disturbance constants, To turn off pick_up greater than 1000
pick_up=100; %time of new load
mu = 0.01; %friction coefficent between wheels and ground
del_weight=1; %Weight of the delivery

%%%%%%%%%% AFTER THIS SECTION RUN SIMULINK %%%%%%%%%%%%%%%%%%
%% Find difference between state observer 
%Get all observer values
x_hat_values=x_hat.data;
x_hat_position=x_hat_values(:,1);
x_hat_alpha=x_hat_values(:,2);
x_hat_beta=x_hat_values(:,3);
x_hat_wr=x_hat_values(:,4);
x_hat_wl=x_hat_values(:,5);
x_hat_time=x_hat.time;

%Get all state values
beta_out=beta_workspace.data;
alpha_out=alpha_workspace.data;
rho_out=rho.data;
wr_out=wr.data;
wl_out=wl.data;
time_states=rho.time;

%State estimation error
e_pos=rho_out-x_hat_position;
e_alpha=alpha_out-x_hat_alpha;
e_beta=beta_out-x_hat_beta;
e_wr=wr_out-x_hat_wr;
e_wl=wl_out-x_hat_wl;

% Convert all the values back to polar coordinates 
theta_calc=(p_d(3)-beta_out)*(180/pi);
x=p_d(1)-(rho_out.*cosd(theta_calc));
y=p_d(2)-(rho_out.*sind(theta_calc));
theta_final=p_d(3)-(alpha_out+beta_out);
theta_final=rad2deg(theta_final);

% Pull the final x values 
final_x_val=x(end);
final_y_val=y(end);
theta_final_val=theta_final(end);

% Find the difference 
diffx = pos_d(1)-final_x_val;
diffy = pos_d(2)-final_y_val;
difftheta = pos_d(3)-theta_final_val;
fprintf('The final position error in the x direction was: %5f meters\n',[diffx]')
fprintf('The final position error in the y direction was: %5f meters\n',[diffy]')
fprintf('The final position error in the rotation was: %5f degrees\n ',[difftheta]')

%%
figure (1)
subplot(2,2,1)
plot(time_states,rho_out,'-b')
hold on
plot(x_hat_time,x_hat_position,'--r')
hold off
title([noise ' Actual Value vs. Estimated ' position])
xlabel('Time (Seconds)')
ylabel('Rho (Distance)')
legend('Actual Position','Estimated Position')

subplot(2,2,2)
plot(time_states,alpha_out,'-b')
hold on
plot(x_hat_time,x_hat_alpha,'--r')
hold on
plot(time_states,beta_out)
hold on
plot(x_hat_time,x_hat_beta,'--k')
hold off
title([noise ' Actual Value vs. Estimated ' position])
xlabel('Time (Seconds)')
ylabel('Output')
legend('Actual Alpha','Estimated Alpha','Actual Beta','Estimated Beta')

subplot(2,2,3)
plot(time_states,wr_out)
hold on
plot(x_hat_time,x_hat_wr)
hold off
title([noise ' Actual Value vs. Estimated ' position])
xlabel('Time (Seconds)')
ylabel('Angular Velocity (rad/sec)')
legend('Actual Right','Estimated Right')

subplot(2,2,4)
plot(time_states,wl_out)
hold on
plot(x_hat_time,x_hat_wl)
hold off
title([noise ' Actual Value vs. Estimated ' position])
xlabel('Time (Seconds)')
ylabel('Angular Velocity (rad/sec)')
legend('Actual Left','Estimated Left')

figure(2)
subplot(2,1,1)
plot(time_states,e_pos)
hold on
plot(time_states,e_alpha)
hold on
plot(time_states,e_beta)
hold off
title([noise ' Error Between Actual and Estimated States ', position])
legend('Position Error','Alpha Error','Beta Error')
xlabel('Time (seconds)')
ylabel('Error')

subplot(2,1,2)
plot(time_states,e_wr)
hold on
plot(time_states,e_wl)
hold off
title([noise ' Error Between Actual and Estimated States ', position])
legend('Right Angular Velocity Error','Left Angular Velocity Error')
xlabel('Time (seconds)')
ylabel('Error')

figure(3) 
subplot(3,1,1)
plot(time_states,x) 
title(['Position Error for x when Initial: ' num2str(pos_i(1)) ' Desired: ' num2str(pos_d(1))])
xlabel('Time (Seconds)')
ylabel('Distance (m)')
ylim([0 p_d(1)+1])

subplot(3,1,2)
plot(time_states,y)
title(['Position Error for y when Initial: ' num2str(pos_i(2)) ' & Desired: ' num2str(pos_d(2))])
xlabel('Time (Seconds)')
ylabel('Distance (m)')
ylim([0 (p_d(2)+1)])

subplot(3,1,3)
plot(time_states,theta_final)
title(['Position Error for Theta when Initial: ' num2str(pos_i(3)) ' & Desired: ' num2str(pos_d(3)) ])
xlabel('Time (Seconds)')
xlim([-1 500])
ylabel('Theta')

%% Find the pole that provides least amount of overshoot  
% N=5; % number of simulations
% step=linspace(0,50,N); %choose the number as how many to add to each
% close all
% 
% for i=1:N
%     
% %     p_v=[-10 -0.4 -0.1 -0.04 -0.7]; %set as initial condition
% %     p_n= p_v + [0 0 0 -1 0 ].*step(i) %add a tolerance value (step) to previous loop 
% %     k=place(A,B,p_n)
%     p_o=[-48 -54 -29 -30 -38];
%     p_on= p_o + [-1 -1 0 0 0].*step(i)
%     bc_1=place(A',C',p_on);
%     bc=bc_1';
%     ac=A-bc*C;
%     k_z=B-bc*D;
%     %implement the model in simulink for given v_ value
%     simOut(i) = sim('milestone3',... 
%                 'SimulationMode','normal',...
%                 'StopTime','300');
%     %Get data points
%     getoutput=simOut(i).get('logsout'); %pull data from simulation i
%     position=getoutput.get('p').Values.Data; %pull linearized output
%     x_h=getoutput.get('x_hat').Values.Data;
%     beta_mc=getoutput.get('beta').Values.Data;
%     alpha_mc=getoutput.get('alpha').Values.Data;
%     x_h_b=x_h(:,3);
%     x_h_a=x_h(:,2);
%     x_h_p=x_h(:,1);
% %Create plots to see when each is trending to zero to get general idea 
%     figure (1)
%     plot(x_h_b)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)];
%     figure(2)
%     plot(beta_mc)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)]; 
%     figure (3)
%     plot(x_h_a)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)];
%     figure(4)
%     plot(alpha_mc)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)];       
%     figure (5)
%     plot(x_h_p)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)];
%     figure(6)
%     plot(position)
%     hold all
%     legend_labels{i} = ['Run ' num2str(i)];
%     
% end
% % Apply legend for each of the runs
% figure(1)
% legend(legend_labels,'Location','NorthEastOutside');
% figure(2)
% legend(legend_labels,'Location','NorthEastOutside');
% figure (3)
% legend(legend_labels,'Location','NorthEastOutside');
% figure (4)
% legend(legend_labels,'Location','NorthEastOutside');


