clear all;

s = tf('s');
Ts = 0.02;

% Transferencia entrada-salida
P_servo = 127.5/(s^2+10.02*s+128.6);
P_carro = -7.7023/(s*(s+0.6552));
Retardo = (2/Ts - s)/(2/Ts + s);
P = P_servo*pi/180*P_carro

% Representacion en variable de estado
sys_ss = ss(P);
A = sys_ss.A
B = sys_ss.B
C = sys_ss.C
D = sys_ss.D

%% validacion
a = 0.6552;  g_deg = -7.7023*pi/180;  
A = [0 1 0 0;  0 -a g_deg 0;  0 0 0 1;  0 0 -128.6 -10.02];
B = [0; 0; 0; 127.5];   C = [1 0 0 0];   D = 0;

sys_ml = ss(P);                        % tu resultado
sys_ph = ss(A,B,C,D);                  % estados fisicos [p; pdot; theta; thetadot]
T = ctrb(sys_ph)/ctrb(sys_ml);         % x_fis = T*x_ml
norm(T*sys_ml.A/T - sys_ph.A)          % ~ 0 (queda un residuo chico por redondeo)
