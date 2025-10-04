
%بسم الله الرحمن الرحيم 
clc
clear all
close all

%this script visualize an invertedflywheel
%defining constants
g = 9.80665; %gravity acceleration
l = 0.1;     % arm length till the flywheel center  [m]
l_g = l/2;   % center of mass of the arm  [m]
m1 = 0.1;    % arm mass  [kg]
m2 = 1;      %flywheel mass   [kg]
r = 0.05;    %flywheel radius  [m]
I1 = (1/12) * m1 * (l)^2;  %arm inertia
I2 = (1/2) * m2 * r^2;     %flywheel inetia
c1 = 1*10^-3; % damping coeffecient between the base and the arm 
c2 = 1*10^-3;  % damping coeffecient between the arm and the flywheel

%state-space representation derived from equation of motion
D=(m1*l_g^2+m2*l^2+I1);

M1=g*(m1*l_g+m2*l);
M2=1 + I1/I2 + m1*l_g^2/I2 + m2*l^2/I2;

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     (M1/D), 0, (-c1/D), (c2/D);
     (-M1/D), 0, (c1/D), (-c2*M2/D)];

B = [0; 0; (-1/D); (M2/D)];
C = [1, 1, 0, 0];

%implementing LQR controller by defining Q and R
Q = [10, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 0.01, 0;
     0, 0, 0, 0.01];
R = 0.01;
K = lqr(A, B, Q, R);
% シミュレーション
dt = 0.002;
t = 0 : dt : 0.7;
x0 = [pi/6; 0; 0; -30]; % 初期値
u = 0; % 入力の初期値
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t
    dx = A * x + B * u;
    x = x + dx * dt;
    u = -K*x;
    
    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end
y=[s_x1',s_x2',s_x3',s_x4'];
T='LQR';
draw42(y,l,r,T,s_u',dt)

figure
subplot(2, 2, 1)
plot(t, y(:,1));
legend('th_1')

subplot(2, 2, 3)
plot(t, y(:,2));
legend('th_2')

subplot(2, 2, 2)
plot(t, y(:,3));
legend('dth_1')

subplot(2, 2, 4)
plot(t, y(:,4));
legend('dth_2')


figure
plot(t,s_u);
xlabel('time(s)')
ylabel('control input u')
legend('u')



