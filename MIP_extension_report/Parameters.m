clear;close all;
%% SET INITIAL ANGLE OF MIP BODY
theta0 = 30*pi/180;
%% PARAMETERS
% physical constants
r = 0.034; % wheel radius m
l = 0.036; % center of mass to axle m
m_w = 0.027; % wheel mass kg
m_b = 0.263; % body mass kg
G_r = 35.57; % gearbox ratio
I_m = 3.6e-8; % motor armature moment of inertia kgm^2
I_b = 4e-4; % body moment of inertia
V_max = 7.4; % max battery voltage V
s_bar = 0.003; % motor stall torque Nm
omega_f = 1760; % motor free run speed rad/sec
g = 9.8; % acceleration due to gravity m/s^2
i=sqrt(-1);

% derived quantities
I_w = m_w*r^2/2+G_r^2*I_m; % interia of singel wheel plus gearbox
k = s_bar/omega_f; % motor constant

% intermediate cosntants
a = 2*I_w+(m_b+2*m_w)*r^2;
b = m_b*r*l;
c = I_b+m_b*l^2;
d = m_b*g*l;
e = 2*G_r*s_bar/V_max;
j = 2*G_r^2*k;
XX = 1/(a*c-b*b);

% linearized matrices
A = [-(a+b)*j*XX (a+b)*j*XX a*d*XX;(b+c)*j*XX -(b+c)*j*XX -b*d*XX;1 0 0];
B = [-(a+b)*e*XX; (b+c)*e*XX; 0];
C = [1 0 0;0 1 0];
D = [0; 0];

%% SYSTEM IDENTIFICATION
% theta
s = tf("s");
tf_exp = -34.81*s/(s^2+6.432*s+99.54);
A_theta = [0 1;-d/c -j/c];
B_theta = [0;-e/c];
C_theta = [0 1];
D_theta = 0;
[num_theta,den_theta] = ss2tf(A_theta,B_theta,C_theta,D_theta);
tf_mdl = tf(num_theta, den_theta);
figure(1);
subplot(2,2,1)
bode(tf_mdl,tf_exp);
legend("mdl","exp");
title("Bode : V -> theta");
grid on;
subplot(2,2,2);
step(tf_mdl,tf_exp);
legend("mdl","exp");
subtitle("V -> theta");
grid on;

% phi
a_phi = 2*I_w;
A_phi = [0 1;0 -j/a_phi];
B_phi = [0;e/a_phi];
C_phi = [0 1];
D_phi = 0;
[num_phi, den_phi] = ss2tf(A_phi,B_phi,C_phi,D_phi);
tf_mdl = tf(num_phi,den_phi);
tf_exp = 252.7*s/(s^2+45.24*s);
subplot(2,2,3);
bode(tf_mdl,tf_exp);
legend("mdl","exp");
title("Bode : V -> phi");
grid on;
subplot(2,2,4);
step(tf_mdl,tf_exp);
legend("mdl","exp");
subtitle("V -> phi");
grid on;

% identified model
I_b = 5.91*1e-4;
k = 2.37*1e-6;
s_bar = 0.003375;
I_w = 6.42*1e-5;

%% UPDATE PARAMETERS
a = 2*I_w+(m_b+2*m_w)*r^2;
b = m_b*r*l;
c = I_b+m_b*l^2;
d = m_b*g*l;
e = 2*G_r*s_bar/V_max;
j = 2*G_r^2*k;
XX = 1/(a*c-b*b);

% full linearized matrices
A = [-(a+b)*j*XX (a+b)*j*XX a*d*XX;(b+c)*j*XX -(b+c)*j*XX -b*d*XX;1 0 0];
B = [-(a+b)*e*XX; (b+c)*e*XX; 0];
C = [1 0 0;0 1 0;0 0 1];
D = [0;0;0];
linearsys = ss(A,B,C,D);
[num, den] = ss2tf(A,B,C,D);
sys_tf_thetadot = tf(num(1,:),den);
sys_tf_phidot = tf(num(2,:),den);
sys_tf_theta = tf(num(3,:),den);
sys_tf_phi = sys_tf_phidot/s;
figure(2);
subplot(2,2,1);
rlocus(sys_tf_thetadot);
subtitle("theta dot");
subplot(2,2,2);
rlocus(sys_tf_phidot);
subtitle("phi dot");
subplot(2,2,3);
rlocus(sys_tf_theta);
subtitle("theta");
subplot(2,2,4);
rlocus(sys_tf_phi);
subtitle("phi");
