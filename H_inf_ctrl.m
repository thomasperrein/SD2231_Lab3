%% This is a Matlab file for designing H_infinity controller 
%% AND SKYHOOK for 2DOF SYSTEMS (assignment 3 of SD2231)
clear all
s=tf('s');

% system parameters
m=22000;   %kg
j=700e3;   %kgm^2
c=40e3;    %Ns/m
k=2*300e3; %N/m
L=6;       %m

%% New parameters
% m=22000*1.15;
% j=700e3*1.15;
% c=40e3*0.85;
% k=2*300e3*0.85;
% L=6;

%% State space model for skyhook contorl
Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];
Csk=[0 1 0 0
    0 0 0 1];
Dsk=zeros(2,4);

%% Plot to find the resonance frequency of the system (to penalyse it the most)
% sys = ss(Ask,Bsk,Csk,Dsk);
% bodemag(sys)

%% H_inf using linmod syntax

%state space: The same as skyhook

      
%Weighting functions

%For penalizing actuator force
Wa1=(0.00175*s+1)/(0.00025*s+1);
Wa2=Wa1;

%For penalizing bounce and pitch motions
eps=1;
wnb=7.39;            %Find the right equation or value for wnb
wnchi=7.86;          %Find the right equation or value for wnchi
s1b=-eps+1i*sqrt(wnb^2-eps^2);
s2b=-eps-1i*sqrt(wnb^2-eps^2);
s1chi=-eps+1i*sqrt(wnchi^2-eps^2);
s2chi=-eps-1i*sqrt(wnchi^2-eps^2);
kb=9e3; %best value found
kchi=4e4; %best value found
Wb=(kb*s1b*s2b)/((s-s1b)*(s-s2b));
Wchi=(kchi*s1chi*s2chi)/((s-s1chi)*(s-s2chi));

%Extracting the extended model
[A_Pe,B_Pe,C_Pe,D_Pe] = linmod('Extended_model');% state space parameters of the extended system: Pe
Pe=ss(A_Pe,B_Pe,C_Pe,D_Pe);

%Calculating the controller
ncont = 2;%Number of control inputs
nmeas = 2;%Number of measured outputs provided to the controller
Pe=minreal(Pe);%This syntax cancels pole-zero pairs in transfer
%functions. The output system has minimal order and the same response
%characteristics as the original model.
[K,Pec,gamma,info]=hinfsyn(Pe,nmeas,ncont,'method','lmi'); % for working with the error
[Ainf, Binf, Cinf, Dinf]=ssdata(K);

%Now use the controller K in your simulation

%% Other parameters for the other 2DOF systems
m_p = 0.16;
c_p = 0.8;
k_p = 6.32;

m_s = 0.16;
c_s = 0.05;
k_s = 0.0632; 

w_n = sqrt(k_p/m_p); 

T = 0.1;
c_z = 1000000;
c_x =3300000;

%% Plot of the bode diagram for question 6.2
% ZsZw = tf([(c_s*c_p) (c_s*k_p+k_s*c_p) (k_s*k_p)],[(m_s*m_p) (m_s*c_s+m_s*c_p+m_p*c_s) (m_s*k_s+m_s*k_p+c_s*c_p+k_s*m_p) (c_s*k_p+k_s*c_p) (k_s*k_p)]); 
% bode(ZsZw) 
% grid on 
% title('Bode diagram for 2DOF system with passive suspension') 
% legend('Damped passive system')


%% Plot of the magnitude of the weighting function
% figure
% bodemag(Wb)
% hold on
% bodemag(Wchi)
% hold off
% title("magnitude of the weighting function")
% xlabel("frequency")
% ylabel("magnitude")
% legend("Weighted function for bounce", "Weighted function for pitch")

%% Plot of the bounce figure
% figure
% plot(out.hinf1.Time,out.hinf1.Data)
% hold on
% plot(out.passive1.Time,out.passive1.Data)
% plot(out.skyhook1.Time,out.skyhook1.Data)
% hold off
% xlabel("Time [s]")
% ylabel("Bounce displacement [m]")
% title("Bounce displacement for passive damped system, skyhook controller and Hinf controller")
% legend("Hinf","Passive damped system","Skyhook")

%% Plot of the pitch figure
% figure
% plot(out.hinf.Time,out.hinf.Data)
% hold on
% plot(out.passive.Time,out.passive.Data)
% plot(out.skyhook.Time,out.skyhook.Data)
% hold off
% xlabel("Time [s]")
% ylabel("Pitch [rad]")
% title("Pitch for passive damped system, skyhook controller and Hinf controller")
% legend("Hinf","Passive damped system","Skyhook")

%% Plot of the forces 
% figure
% plot(out.fa1Hinf.Time,out.fa1Hinf.Data)
% hold on
% plot(out.fa2Hinf.Time,out.fa2Hinf.Data)
% plot(out.fa1sky.Time,out.fa1sky.Data)
% plot(out.fa2sky.Time,out.fa2sky.Data)
% plot([out.fa1Hinf.Time(1),out.fa1Hinf.Time(end)],[10000,10000],'b-')
% plot([out.fa1Hinf.Time(1),out.fa1Hinf.Time(end)],[-10000,-10000],'b-')
% hold off
% xlabel("Time [s]")
% ylabel("Force [N]")
% title("Forces in the actuators over time")
% legend("Fa1 Hinf","Fa2 Hinf","Fa1 Skyhook","Fa2 Skyhook")