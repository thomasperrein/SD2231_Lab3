%% Parameters
mp=0.16; %[kg]
cp=0.4; %[Ns/m]
kp=6.32; %[N/m]

%% Task 1.3
s=tf('s');
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
