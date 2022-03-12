close all;
%% Parameters
Cz = [eye(3);zeros(1,3)];
Dzu = [zeros(3,1);1];
w = 4;

% stablizable:
[U,T] = schur(A);
[US,TS] = ordschur(U,T,'lhp');
disp("TS = ");
disp(TS);
BU = US'*B;
disp("B = ");
disp(BU);
thetadot_noise = 0.01;
phidot_noise = 0.1;
theta_noise = 0.1;

%% LQR control by feedback theta
disp("-------------------------------------------------------");
disp("LQR control by feedback theta :")
disp(" ");
Cy_theta = [0 0 1]; % feedback theta
% detectable
CU = Cy_theta*US;
disp("detectable for Cy = [0 0 1] :");
disp(CU);
v = theta_noise; % covariance
W = blkdiag(w,v);
Bw_theta = [B,zeros(3,1)];
Dyw_theta = [0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_theta = -K
[F,~,~] = lqr(A',Cy_theta',Bw_theta*W*Bw_theta',Dyw_theta*W*Dyw_theta');
F_theta = -F'
A_ctr = A+B*K_theta+F_theta*Cy_theta;
B_ctr = -F_theta;
C_ctr = K_theta;
D_ctr = 0;
[num, den] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr);
H = tf(num,den);
H_lqgctr_theta = feedback(H,-sys_tf_theta);
tf_thetadot_lqgctrbytheta = minreal(H_lqgctr_theta*sys_tf_thetadot);
tf_phidot_lqgctrbytheta = minreal(H_lqgctr_theta*sys_tf_phidot);
figure(1);
subplot(3,1,1);
pzmap(H_lqgctr_theta);
title("poles and zeros of theta dot");
subplot(3,1,2);
pzmap(tf_thetadot_lqgctrbytheta);
title("poles and zeros of theta dot");
subplot(3,1,3);
pzmap(tf_phidot_lqgctrbytheta);
title("poles and zeros of phi dot");
[~,gain_theta] = zero(H);
disp("Gain :");
disp(gain_theta);

%% control by feedback thetadot
disp("-------------------------------------------------------");
disp("LQR control by feedback theta dot :")
disp(" ");
Cy_thetadot = [1 0 0]; % feedback theta
% detectable
CU = Cy_thetadot*US;
disp("detectable for Cy = [1 0 0] :");
disp(CU);
v = thetadot_noise; % covariance
W = blkdiag(w,v);
Bw_thetadot = [B,zeros(3,1)];
Dyw_thetadot = [0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_thetadot = -K
[F,~,~] = lqr(A',Cy_thetadot',Bw_thetadot*W*Bw_thetadot',Dyw_thetadot*W*Dyw_thetadot');
F_thetadot = -F'
A_ctr = A+B*K_thetadot+F_thetadot*Cy_thetadot;
B_ctr = -F_thetadot;
C_ctr = K_thetadot;
D_ctr = 0;
[num, den] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr);
H = tf(num,den);
H_lqgctr_thetadot = feedback(H,-sys_tf_thetadot);
tf_theta_lqgctrbythetadot = minreal(H_lqgctr_thetadot*sys_tf_theta);
tf_phidot_lqgctrbythetadot = minreal(H_lqgctr_thetadot*sys_tf_phidot);
figure(2);
subplot(3,1,1);
pzmap(H_lqgctr_thetadot);
title("poles and zeros of theta dot");
subplot(3,1,2);
pzmap(tf_theta_lqgctrbythetadot);
title("poles and zeros of theta dot");
subplot(3,1,3);
pzmap(tf_phidot_lqgctrbythetadot);
title("poles and zeros of phi dot");
[~,gain_thetadot] = zero(H);
disp("Gain :");
disp(gain_thetadot);

%% control by feedback phidot
disp("-------------------------------------------------------");
disp("LQR control by feedback phi dot :")
disp(" ");
Cy_phidot = [0 1 0]; % feedback theta
% detectable
CU = Cy_phidot*US;
disp("detectable for Cy = [0 1 0] :");
disp(CU);
v = phidot_noise; % covariance
W = blkdiag(w,v);
Bw_phidot = [B,zeros(3,1)];
Dyw_phidot = [0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_phidot = -K
[F,~,~] = lqr(A',Cy_phidot',Bw_phidot*W*Bw_phidot',Dyw_phidot*W*Dyw_phidot');
F_phidot = -F'
A_ctr = A+B*K_phidot+F_phidot*Cy_phidot;
B_ctr = -F_phidot;
C_ctr = K_phidot;
D_ctr = 0;
[num, den] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr);
H = tf(num,den);
H_lqgctr_phidot = feedback(H,-sys_tf_phidot);
tf_theta_lqgctrbyphidot = minreal(H_lqgctr_phidot*sys_tf_theta);
tf_thetadot_lqgctrbyphidot = minreal(H_lqgctr_phidot*sys_tf_thetadot);
figure(3);
subplot(3,1,1);
pzmap(H_lqgctr_phidot);
title("poles and zeros of theta dot");
subplot(3,1,2);
pzmap(tf_theta_lqgctrbyphidot);
title("poles and zeros of theta dot");
subplot(3,1,3);
pzmap(tf_thetadot_lqgctrbyphidot);
title("poles and zeros of phi dot");
[~,gain_phidot] = zero(H);
disp("Gain :");
disp(gain_phidot);

%% LQR control by feedback thetadot and phidot
disp("-------------------------------------------------------");
disp("LQR control by feedback thetadot and phidot :")
disp(" ");
Cy_thetadot_phidot = [1 0 0;0 1 0]; % feedback theta
% detectable
CU = Cy_thetadot_phidot*US;
disp("detectable for Cy = [1 0 0;0 1 0] :");
disp(CU);
v = [thetadot_noise 0;0 phidot_noise]; % covariance
W = blkdiag(w,v);
Bw_thetadot_phidot = [B,zeros(3,2)];
Dyw_thetadot_phidot = [0 1 0;0 0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_thetadot_phidot = -K
[F,~,~] = lqr(A',Cy_thetadot_phidot',Bw_thetadot_phidot*W*Bw_thetadot_phidot',Dyw_thetadot_phidot*W*Dyw_thetadot_phidot');
F_thetadot_phidot = -F'
A_ctr = A+B*K_thetadot_phidot+F_thetadot_phidot*Cy_thetadot_phidot;
B_ctr = -F_thetadot_phidot;
C_ctr = K_thetadot_phidot;
D_ctr = [0 0];
[num_thetadot, den_thetadot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,1);
H_thetadot = tf(num_thetadot,den_thetadot);
[num_phidot, den_phidot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,2);
H_phidot = tf(num_phidot,den_phidot);
[~,gain_thetadot_phidot_thetadot] = zero(H_thetadot);
[~,gain_thetadot_phidot_phidot] = zero(H_phidot);
disp("Gain (thetadot):");
disp(gain_thetadot_phidot_thetadot);
disp("Gain (phidot):");
disp(gain_thetadot_phidot_phidot);

%% LQG control by feedback theta and thetadot
disp("-------------------------------------------------------");
disp("LQR control by feedback theta and thetadot :")
disp(" ");
Cy_thetadot_theta = [1 0 0;0 0 1]; % feedback theta
% detectable
CU = Cy_thetadot_theta*US;
disp("detectable for Cy = [1 0 0;0 0 1] :");
disp(CU);
v = [0.1 0;0 0.01]; % covariance
W = blkdiag(w,v);
Bw_thetadot_theta = [B,zeros(3,2)];
Dyw_thetadot_theta = [0 1 0;0 0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_thetadot_theta = -K
[F,~,~] = lqr(A',Cy_thetadot_theta',Bw_thetadot_theta*W*Bw_thetadot_theta',Dyw_thetadot_theta*W*Dyw_thetadot_theta');
F_thetadot_theta = -F'
A_ctr = A+B*K_thetadot_theta+F_thetadot_theta*Cy_thetadot_theta;
B_ctr = -F_thetadot_theta;
C_ctr = K_thetadot_theta;
D_ctr = [0 0];
[num_thetadot, den_thetadot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,1);
H_thetadot = tf(num_thetadot,den_thetadot);
[num_theta, den_theta] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,2);
H_theta = tf(num_theta,den_theta);
[~,gain_thetadot_theta_thetadot] = zero(H_thetadot);
[~,gain_thetadot_theta_theta] = zero(H_theta);
disp("Gain (thetadot):");
disp(gain_thetadot_theta_thetadot);
disp("Gain (theta):");
disp(gain_thetadot_theta_theta);

%% LQG control by feedback phidot and theta
disp("-------------------------------------------------------");
disp("LQR control by feedback phidot and theta :")
disp(" ");
Cy_phidot_theta = [0 1 0;0 0 1]; % feedback theta
% detectable
CU = Cy_phidot_theta*US;
disp("detectable for Cy = [0 1 0;0 0 1] :");
disp(CU);
v = [0.1 0;0 0.01]; % covariance
W = blkdiag(w,v);
Bw_phidot_theta = [B,zeros(3,2)];
Dyw_phidot_theta = [0 1 0;0 0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_phidot_theta = -K
[F,~,~] = lqr(A',Cy_phidot_theta',Bw_phidot_theta*W*Bw_phidot_theta',Dyw_phidot_theta*W*Dyw_phidot_theta');
F_phidot_theta = -F'
A_ctr = A+B*K_phidot_theta+F_phidot_theta*Cy_phidot_theta;
B_ctr = -F_phidot_theta;
C_ctr = K_phidot_theta;
D_ctr = [0 0];
[num_phidot, den_phidot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,1);
H_phidot = tf(num_phidot,den_phidot);
[num_theta, den_theta] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,2);
H_theta = tf(num_theta,den_theta);
[~,gain_phidot_theta_phidot] = zero(H_phidot);
[~,gain_phidot_theta_theta] = zero(H_theta);
disp("Gain (phidot):");
disp(gain_phidot_theta_phidot);
disp("Gain (theta):");
disp(gain_phidot_theta_theta);

%% LQG control by feedback thetadot, phidot and theta
disp("-------------------------------------------------------");
disp("LQR control by feedback thetadot, phidot and theta :")
disp(" ");
Cy_all = [0 1 0;0 1 0;0 0 1]; % feedback theta
% detectable
CU = Cy_all*US;
disp("detectable for Cy = [0 1 0;0 0 1] :");
disp(CU);
v = [0.1 0 0;0 0.01 0;0 0 0.01]; % covariance
W = blkdiag(w,v);
Bw_all = [B,zeros(3,3)];
Dyw_all = [0 1 0 0;0 0 1 0;0 0 0 1];
[K,~,~] = lqr(A,B,Cz'*Cz,Dzu'*Dzu);
K_all = -K
[F,~,~] = lqr(A',Cy_all',Bw_all*W*Bw_all',Dyw_all*W*Dyw_all');
F_all = -F'
A_ctr = A+B*K_all+F_all*Cy_all;
B_ctr = -F_all;
C_ctr = K_all;
D_ctr = [0 0 0];
[num_thetadot, den_thetadot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,1);
H_thetadot = tf(num_thetadot,den_thetadot);
[num_phidot, den_phidot] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,2);
H_phidot = tf(num_phidot,den_phidot);
[num_theta, den_theta] = ss2tf(A_ctr,B_ctr,C_ctr,D_ctr,3);
H_theta = tf(num_theta,den_theta);
[~,gain_all_thetadot] = zero(H_thetadot);
[~,gain_all_phidot] = zero(H_phidot);
[~,gain_all_theta] = zero(H_theta);
disp("Gain (thetadot):");
disp(gain_all_thetadot);
disp("Gain (phidot):");
disp(gain_all_phidot);
disp("Gain (theta):");
disp(gain_all_theta);
