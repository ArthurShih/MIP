close all;
%% CONTROL BY FEEDBACK THETA
figure(1);
subplot(2,2,1);
rlocus(sys_tf_theta);
title("theta root locus");
K = tf(-(s+6.5)/(s-0.5));

subplot(2,2,2);
rlocus(K*sys_tf_theta);
title("root locus with K = -(s+6.5)/(s-0.5)");

subplot(2,2,3);% find proper gain to stable theta
for G = 4:2:10
    pzmap(feedback(G*K*sys_tf_theta,1));
    hold on;
end
title("poles and zeros under different gain")
legend("4","6","8","10")
hold off;

subplot(2,2,4);
for G = 4:2:10
    step(feedback(G*K*sys_tf_theta,1));
    hold on;
end
title("step response under different gain");
legend("4","6","8","10");
hold off

G = 5.2; % Gain
K_cctr_theta = tf(G*K);
H_cctr_theta = feedback(K_cctr_theta,sys_tf_theta);
tf_thetadot_cctrbytheta = minreal(H_cctr_theta*sys_tf_thetadot);
tf_phidot_cctrbytheta = minreal(H_cctr_theta*sys_tf_phidot);
figure(2);
subplot(3,1,1);
pzmap(H_cctr_theta);
title("poles and zeros of theta");
subplot(3,1,2);
pzmap(tf_thetadot_cctrbytheta);
title("poles and zeros of theta dot");
subplot(3,1,3);
pzmap(tf_phidot_cctrbytheta);
title("poles and zeros of phi dot");

disp("Classical control by feedback theta :");
K_cctr_theta
disp("----------------------------------------------------");
%% CONTROL BY FEEDBACK THETA DOT
figure(3);
subplot(2,2,1);
rlocus(sys_tf_thetadot);
title("thetadot root locus");
K = -1/(s-26.5);

subplot(2,2,2);
rlocus(K*sys_tf_thetadot);
title("root locus with K = -1/(s-26.5)");
subplot(2,2,3); % find proper gain to stable theta dot
for G = 20:2:30
    pzmap(feedback(G*K*sys_tf_thetadot,1));
    hold on;
end
title("poles and zeros under different gain")
legend("20","22","24","26","28","30")
hold off;
subplot(2,2,4);
for G = 20:2:30
    step(feedback(G*K*sys_tf_thetadot,1));
    hold on;
end
title("step response under different gain");
legend("20","22","24","26","28","30");
hold off

G = 22; % Gain
K_cctr_thetadot = tf(G*K);
H_thetadot = feedback(K_cctr_thetadot,sys_tf_thetadot);
tf_theta_ctrbythetadot = minreal(H_thetadot*sys_tf_theta);
tf_phidot_ctrbythetadot = minreal(H_thetadot*sys_tf_phidot);
figure(4);
subplot(3,1,1);
pzmap(H_thetadot);
title("poles and zeros of theta dot");
subplot(3,1,2);
pzmap(tf_theta_ctrbythetadot);
title("poles and zeros of theta");
subplot(3,1,3);
pzmap(tf_phidot_ctrbythetadot);
title("poles and zeros of phi dot");

disp("Classical control by feedback thetadot :");
K_cctr_thetadot
disp("----------------------------------------------------");
%% Control by thetadot and phidot
figure(5);
subplot(3,1,1);
G_thetadot_phidot = minreal(H_thetadot*sys_tf_phidot);
rlocus(G_thetadot_phidot);
title("root locus of closed thetadot to phidot");
subplot(3,1,2);
for G = 0.01:0.01:0.05
    pzmap(feedback(G*G_thetadot_phidot,1));
    hold on;
end
hold off;
legend("0.01","0.02","0.03","0.04","0.05");
title("poles and zeros under different gain");
subplot(3,1,3);
for G = 0.01:0.01:0.05
    step(feedback(G*G_thetadot_phidot,1));
    hold on;
end
hold off;
legend("0.01","0.02","0.03","0.04","0.05");
title("step response under different gain");

G = 0.01;
K_cctr_thetadot_phidot = G;
H_thetadot_phidot = minreal(feedback(K_cctr_thetadot_phidot*H_thetadot,sys_tf_phidot));
figure(6);
subplot(3,1,1);
pzmap(minreal(H_thetadot_phidot*sys_tf_theta));
title("poles and zeros of theta dot");
subplot(3,1,2);
pzmap(H_thetadot_phidot);
title("poles and zeros of phidot");
subplot(3,1,3);
pzmap(minreal(H_thetadot_phidot*sys_tf_theta));
title("poles and zeros of theta");

disp("Classical control by feedback thetadot and phidot :");
K_cctr_thetadot_phidot
disp("----------------------------------------------------");