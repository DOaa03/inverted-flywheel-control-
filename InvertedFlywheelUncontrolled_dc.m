%بسم الله الرحمن الرحيم 
%clc
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
c1 = 5*10^-2; % damping coeffecient between the base and the arm 
c2 = 1*10^-3;  % damping coeffecient between the arm and the flywheel

R = 1.2;          % [Ohm]
L = 1.8e-3;       % [H]
Kt = 0.055;       % [Nm/A]
Kv = 0.055;       % [V/(rad/sec)]

Te=L/R;
wc=5/Te;


%state-space representation derived from equation of motion
D=(m1*l_g^2+m2*l^2+I1);

M1=g*(m1*l_g+m2*l);
M2=1 + I1/I2 + m1*l_g^2/I2 + m2*l^2/I2;
M3=(Kt)/R;
M4=(Kt*Kv)/R;

%setting for simulation
dt = 0.01;
t = 0 : dt : 10;
x0 = [1; 0; 0; 0]; 
u = 0; 
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t

    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

    sy=sin(x(1));
    D=(m1*l_g^2+m2*l^2+I1);
    M1=g*(m1*l_g+m2*l);
    M2=1 + I1/I2 + m1*l_g^2/I2 + m2*l^2/I2;


    dx(1,1)=x(3);
    dx(2,1)=x(4);
    dx(3,1)=(M1/D)*sin(x(1)) + (-c1/D)*x(3) + ((c2+M4)/D)*x(4) - u*(M3/D);
    dx(4,1)=(-M1/D)*sin(x(1)) + (c1/D)*x(3) - ((c2 + M3)*M2/D)*x(4) + u*(M3*M2/D);

    
    x = x + dx * dt 
    
    
    

end
figTitle='uncontrolled';
y=[s_x1',s_x2',s_x3',s_x4'];
draw42(y,l, r, figTitle, s_u', dt)



