clear; 
%close all;
clc;

%this script visualize an invertedflywheel
%defining constants
g = 9.80665; %gravity acceleration
l = 0.1;     % arm length till the flywheel center  [m]
l_g = l/2;   % center of mass of the arm  [m]
m1 = 0.1;    % arm mass  [kg]
m2 = 1;      %flywheel mass   [kg]
r_fw= 0.05;    %flywheel radius  [m]
I1 = (1/12) * m1 * (l)^2;  %arm inertia
I2 = (1/2) * m2 * r_fw^2;     %flywheel inetia
c1 = 1*10^-3; % damping coeffecient between the base and the arm 
c2 = 1*10^-3;  % damping coeffecient of the dc motor

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


A = [0, 0, 1, 0;
     0, 0, 0, 1;
     (M1/D), 0, (-c1/D), (c2+M4/D);
     (-M1/D), 0, (c1/D), (-((c2+M4)*M2)/D)];

B = [0; 0; (-M3/D); ((M2*M3)/D)];
C = eye(4);
D_sys = zeros(4,1);

ts = 0.005; % sampling time
plant = ss(A,B,C,D_sys);
planted = c2d(plant,ts,'zoh');

% MPC setup 
Prediction_horizon = 100;
Control_horizon = 20;
mpc_obj = mpc(planted,ts,Prediction_horizon, Control_horizon);

mpc_obj.MV.Min = -24;
mpc_obj.MV.Max = 24;
mpc_obj.Model.Plant.InputName = 'Motor Volt';
mpc_obj.Model.Plant.InputUnit = 'v';

mpc_obj.Model.Nominal.X = [0;0;0;0];
mpc_obj.Model.Nominal.Y = [0;0;0;0];
mpc_obj.Model.Nominal.U = 0;
mpc_obj.Model.Nominal.DX = [0;0;0;0];

mpc_obj.Model.Plant.OutputName = {'theta1','theta2','theta1_dot','theta2_dot'};
mpc_obj.Model.Plant.OutputUnit = {'rad','rad','rad/s','rad/s'};

% constraints on theta (small angle)
mpc_obj.OutputVariables(1).Min = -0.05;
mpc_obj.OutputVariables(1).Max =  0.05;
mpc_obj.OutputVariables(1).MinECR = 1;
mpc_obj.OutputVariables(1).MaxECR = 1;

%constrains on change in voltage every step
max_dV_per_sampe= 1.24;
mpc_obj.MV(1),RateMin = -max_dV_per_sampe;
mpc_obj.MV(1),RateMax =  max_dV_per_sampe;

% weights
mpc_obj.Weights.OutputVariables = [200 0 200 50];
mpc_obj.Weights.MV = 5;
mpc_obj.Weights.MVRate = 0.3;

% initial
initial_state = [-pi/8; 0; 0; 0]; % {'theta1','theta2','theta1_dot','theta2_dot'}
state = mpcstate(mpc_obj);
state.Plant = initial_state;

t = 0:ts:2;
N = length(t);
y = zeros(N,4);
u = zeros(N,1);
r = zeros(4,1);

for k=1:N
    % switching controller
    y(k,:) = state.Plant.';

    if abs(y(k,1)) < (15*pi/180) 
       
        u(k) = mpcmove(mpc_obj, state, y(k,:).', r);  % column vector expected
        % simulate plant one step
        state.Plant = planted.A * state.Plant + planted.B * u(k);
    else
        E_target =M1 ;
        K_gain = 50;     % Adjust this gain for performance

        % Calculate the current energy of the system
        E_current = M1* cos(y(k,1)) + 0.5*D*y(k,3)^2 +0.5*I2*(y(k,3)+y(k,4))^2;

        % Calculate the torque based on the energy difference
        u(k)=  K_gain * (E_current - E_target) *sign( y(k,3));
 

        d_theta_1=y(k,3);
        d_theta_2=y(k,4);
        dd_theta_1 = (M1/D)*sin(y(k,1)) + (-c1/D)*y(k,3) + ((c2+M4)/D)*y(k,4) - u(k)*(M3/D);
        dd_theta_2 = (-M1/D)*sin(y(k,1)) + (c1/D)*y(k,3) - ((c2 + M3)*M2/D)*y(k,4) + u(k)*(M3*M2/D);

         % Euler integration to get the next state
         dx =[d_theta_1;d_theta_2;dd_theta_1;dd_theta_2];
        next_state = state.Plant + dx * ts ;
        
        % Update the plant state for the next iteration
        state.Plant = next_state;
        
        
    end
    % The angle wrapping should use the updated state.Plant
    state.Plant(1) = atan2(sin(state.Plant(1)), cos(state.Plant(1))); 
end



Title='MPC';
drawIF(y,l,r_fw,Title,u,ts)
figure
plot(t,y(:,1),'LineWidth',2)
legend('$\theta_1');
figure
plot(t,y(:,2),'LineWidth',2)
legend('$\theta_2');
figure
plot(t,y(:,3),'LineWidth',2)
legend('$dot{\theta_1}');
figure
plot(t,y(:,4),'LineWidth',2)
legend('$dot{\theta_2}')

figure 
plot(t,u,'linewidth',3)
