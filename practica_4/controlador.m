% Para la planta
%   -1.6098 (s-95.29)
%  ----------------------
%  (s^2 + 13.22s + 151.6)
close all; 
s = tf('s');

alpha_eq = 0;

P_c = tf(-1.6098*(s-95.29)/(s^2 + 13.22*s + 151.6));

kp = 0.6; ki = 6;
C = tf(kp*(s + ki/kp)/s);
C_mejor = pidtune(P_c,'PI'); % Kp = 0.57, Ki = 6.64

figure; bode(P_c*C,P_c*C_mejor);


%% Analisis temporal
L = P_c*C;
L_mejor = P_c*C_mejor;

S=1/(1+L); 
S_mejor=1/(1+L_mejor); 
T=1-S;
T_mejor=1-S_mejor;

figure(); step(T,15);title('T');grid on;
figure(); step(T_mejor,15);title('T mejor');grid on;