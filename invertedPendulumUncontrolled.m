% this script chicks the system response with no controlling signal
% it slao varifies the corectness of the mathmatical equations
clear all 
close all
clc
%initializing parameters
m=1; %pendulum mass
M=5; %cart mass
L=2; %pendulum length
g=10; %gravity acceleration
d=3; %damping coefficient

dt = 0.01;
t = 0 : dt : 500;
x0 = [1; 0; 15*pi/180; 0.05]; % 初期値
u = 0; % 入力の初期値
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t
    cx=cos(x(3));
    sx=sin(x(3));
    D=m*L*L*(M+m*(1-cx^2));

    dx(1,1)=x(2);
    dx(2,1)=(1/D)*(-m^2*L^2*g*cx*sx+m*L^2*(m*L*x(4)^2*sx-d*x(2)))+m*L^2*(1/D)*u;
    dx(3,1)=x(4);
    dx(4,1)=(1/D)*(m*g*L*sx*(m+M)-m*L*cx*(m*L*x(4)^2*sx-d*x(2)))-m*L*cx*(1/D)*u;
  
    x = x + dx * dt;

    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end

T1='no control';
y=[s_x1',s_x2',s_x3',s_x4'];
draw4(y,L,T1,s_u,dt)