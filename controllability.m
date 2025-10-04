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

%checking controlability of the system

controliability_matrix=ctrb(A,B);
Rank=rank(controliability_matrix);

if Rank== length(A)
    disp('this system is controllable')
else
    disp('this uncontrollable')
end
