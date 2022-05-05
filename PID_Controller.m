%% Parameters
mp=0.16; %[kg]
cp=0.4; %[Ns/m]
kp=6.32; %[N/m]
s=tf('s');

%% Task 1.3
zeta=cp/(2*sqrt(kp*mp));
omega_n=sqrt(kp/mp);

ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);
figure
bode(ft)

%% Task 1.4


figure
cp=2*sqrt(kp*mp); %to have zeta=1
zeta=cp/(2*sqrt(kp*mp));
ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);
step(ft)
hold on
cp=4*sqrt(kp*mp); %to have zeta=2>1
zeta=cp/(2*sqrt(kp*mp));
ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);
step(ft)
cp=sqrt(kp*mp); %to have zeta=0.5<1
zeta=cp/(2*sqrt(kp*mp));
ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);
step(ft)
legend('zeta=1','zeta=2','zeta=0.5')

%% Task 2
close all
dd=1; %corrector PD parameters 
dp=0; %value of the answer dd=1, dp=0; value critical dd=2.011, dp=0

zeta2=dd/(2*sqrt(mp*(kp+dp)));
wn2 = sqrt((kp+dp)/mp);

display(zeta2)
display(wn2)

ft2=kp/(mp*s^2+s*dd+(dp+kp));

%to compare
zeta=cp/(2*sqrt(kp*mp));
omega_n=sqrt(kp/mp);
ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);

%Plot of both impulse response
t1=linspace(0,10,1000);
I=find(t1>=0.1);
impulse_funct=zeros(size(t1));
impulse_funct(1:I(1))=0.05;
x=lsim(ft,impulse_funct,t1);
x2=lsim(ft2,impulse_funct,t1);
figure
% impulse(ft)
plot(t1,x)
hold on
% impulse(ft2)
plot(t1,x2)
legend('damped passive system','active controller')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('Impulse response for both system with dd=1 and dp=0')
hold off

%Plot of sinusoidal excitation   
u = sin(omega_n*t1); 
y = lsim(ft,u,t1);
y2 = lsim(ft2,u,t1);

figure
plot(t1,y)
hold on
plot(t1,y2)
legend('damped passive system','active controller')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('sinusoidal response for both system "sin(wn*t)"')
hold off

%% Task 3
hp=0.001; %parameter of the corrector, good value hp=0.001
hi=3; %good value hi=3
hd=100; %good value hd=100

ft3=kp*s/(mp*s^3+hd*s^2+(dp+hp)*s+hi);

%to compare
zeta=cp/(2*sqrt(kp*mp));
omega_n=sqrt(kp/mp);
ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);

%Plot of both impulse response
x3=lsim(ft3,impulse_funct,t1);
figure
% impulse(ft)
plot(t1,x)
hold on
% impulse(ft3)
plot(t1,x3)
legend('damped passive system','active controller PID')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('Impulse response for both system with hp=0.001 and hd=100 and hi=3')
hold off

%Plot of sinusoidal excitation
y3 = lsim(ft3,u,t1);

figure
plot(t1,y)
hold on
plot(t1,y3)
legend('damped passive system','active controller PID')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('sinusoidal response for both system "sin(wn*t)"')
hold off

%% Tune of Skyhook
mp=0.16; %[kg]
cp=0.4; %[Ns/m]
kp=6.32; %[N/m]
s=tf('s');
T=1.5; %best value T=1.5
ft4=kp/(mp*s^2 + (cp+T)*s+kp); %Skyhook
x4=lsim(ft4,impulse_funct,t1);
figure
plot(t1,x)
hold on
plot(t1,x4)
legend('damped passive system','Skyhook')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('Impulse response for both system with T=1.5')
hold off

%Plot of sinusoidal excitation
y4 = lsim(ft4,u,t1);

figure
plot(t1,y)
hold on
plot(t1,y4)
legend('damped passive system','Skyhook')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('sinusoidal response for both system "sin(wn*t)" with T=1.5')
hold off

%% Comparison of magnitude
mp=0.16; %[kg]
cp=0.4; %[Ns/m]
kp=6.32; %[N/m]
s=tf('s');
zeta=cp/(2*sqrt(kp*mp));
omega_n=sqrt(kp/mp);

dd=1; %corrector PD parameters 
dp=0; %value of the answer dd=1, dp=0; value critical dd=2.011, dp=0

hp=0.001; %parameter of the corrector, good value hp=0.001
hi=3; %good value hi=3
hd=100; %good value hd=100

T=1.5; %skyhook best parameter 1.5

ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2); %damp
ft_undamp=(2*0*omega_n*s+omega_n^2)/(s^2 + 2*0*omega_n*s + omega_n^2); %und tf
ft2=kp/(mp*s^2+s*dd+(dp+kp)); %PD tf
ft3=kp*s/(mp*s^3+hd*s^2+(dp+hp)*s+hi); %PID tf
ft4=kp/(mp*s^2 + (cp+T)*s+kp); %Skyhook

%plot of bode
bode(ft)
hold on
bode(ft_undamp)
bode(ft2)
bode(ft3)
bode(ft4)
legend("damped passive system","undamped","PD controller","PID controller","Skyhook")
hold off

%% Comparison for the first two excitations
figure
plot(t1,x)
hold on
plot(t1,x2)
plot(t1,x3)
plot(t1,x4)
legend('damped passive system','PD','PID','Skyhook')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('Impulse response for all systems')
hold off

figure
plot(t1,y)
hold on
plot(t1,y2)
plot(t1,y3)
plot(t1,y4)
legend('damped passive system','PD','PID','Skyhook')
xlabel('Time [s]')
ylabel('Displacement [m]')
title('sinusoidal response for all systems "sin(wn*t)"')
hold off


%% PSD road excitation
syms w s
v=50; %speed of the train
road=4.028e-7/(2.88e-4 +0.68*w^2 +w^4);
true_road=1/v*road;

ft=(2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s + omega_n^2);
ft2=kp/(mp*s^2+s*dd+(dp+kp));
ft3=kp*s/(mp*s^3+hd*s^2+(dp+hp)*s+hi);
ft4=kp/(mp*s^2 + (cp+T)*s+kp);

freq=[0:0.1:25];
%for the PD controller
ft_w=subs(ft,s,j*w);
ft2_w=subs(ft2,s,j*w);

S1(w)=abs(ft_w)^2*true_road;
S2(w)=abs(ft2_w)^2*true_road;


%for the PID controller
ft3_w=subs(ft3,s,j*w);
S3(w)=abs(ft3_w)^2*true_road;

%for the Skyhook
ft4_w=subs(ft4,s,j*w);
S4(w)=abs(ft4_w)^2*true_road;

%Plot to compare
figure
semilogy(freq,S1(freq))
hold on 
semilogy(freq,S2(freq))
semilogy(freq,S3(freq))
semilogy(freq,S4(freq))
xlabel("frequency in rad/s")
ylabel("power spectral density of the vehicle response w (log)")
title("Comparison between PSD of the vehicle response with and without controller for a range of frequencies")
legend('damped passive system','PD controller','PID Controller','Skyhook')
hold off



