N  = length(tita);                      % cant de muestras
Ts = 0.02;                              % periodo de muestreo
t  = (0:N-1)' * Ts;

g = -9.5;  a = 3;
% funcion de transferencia continua
s = tf('s');
P_c = zpk(g/(s*(s+a)));
polos_c = pole(P_c);

A = [0, 1; 0, -a];
B = [0; g];
C = [1, 0]; 
D = 0;

P_c_ss = ss(A, B, C, D);
pos_sim = lsim(P_c_ss, tita, t, [pos(1); 0]);

figure();plot(t, pos*100, t, pos_sim*100);
legend('medido','modelo'); xlabel('Tiempo [s]'); ylabel('Posicion [m]'); grid on;