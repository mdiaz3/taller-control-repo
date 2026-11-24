s = tf('s');
a=3.28; g=-18.65*pi/180;Ts=0.02;

P_servo = 127.5/(s^2+10.02*s+128.6);
P_carrito = g/(s*(s+a));
P = P_servo * P_carrito;
polos = pole(P);

% Matrices continuas
A = [0 1 0 0; 0 -a g 0; 0 0 0 1; 0 0 -128.6 -10.02];  
B = [0;0;0;127.5];  
C = [1 0 0 0; 0 0 1 0];  
D = zeros(2,1);

%% Controlabilidad
rank(ctrb(A,B))                      % debe dar 4

%% Discretización exacta
sysd = c2d(ss(A,B,C,D), Ts, 'zoh');
[A_d,B_d,C_d,~] = ssdata(sysd);

%% Diseño de K (polos continuos -> discretos)
p_ctl_c = [-6+3j, -6-3j, -10, -11]*1.1;    % punto de partida, a ajustar
p_ctl_d = exp(p_ctl_c*Ts);
K = place(A_d, B_d, p_ctl_d);        % 1x4
disp(K)
disp(abs(eig(A_d - B_d*K))')         % todos < 1

