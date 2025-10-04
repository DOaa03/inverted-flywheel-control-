
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
dt = 0.001;
t = 0 : dt : 10;
x0 = [-pi/8; 0; 0; 0]; % 初期値
u = 0; % 入力の初期値
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t
    

    if abs(x(1)) < (15 * pi/180) 
        % linear control around upright
        y_target = [0;0;0;0];
        u = -K * x ;

        % dynamics (linearized approx around upright)
       
        dydt = A* x  + B * u;
        x = x + dydt * dt;
    else
        E_target =M1 ;
        K_gain = 15;     % Adjust this gain for performance

        % Calculate the current energy of the system
        E_current = M1* cos(x(1)) + 0.5*D*x(3)^2 +0.5*I2*(x(3)+x(4))^2;

        % Calculate the torque based on the energy difference
        u = K_gain * (E_current - E_target) *sign( x(3));

        dydt(1,1)=x(3);
        dydt(2,1)=x(4);
        dydt(3,1) = (M1/D)*sin(x(1)) + (-c1/D)*x(3) + (c2/D)*x(4) - u*(1/D);
        dydt(4,1) = (-M1/D)*sin(x(1)) + (c1/D)*x(3) - (c2*M2/D)*x(4) + u*(M2/D);

        dx =[dydt(1,1);dydt(2,1);dydt(3,1);dydt(4,1)];
        x = x + dx * dt;
        
        
    end
    
   
   
    
    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end
y=[s_x1',s_x2',s_x3',s_x4'];
y_short=y(800:end,:);
%drawFlywheel(y_short,l,r)

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




