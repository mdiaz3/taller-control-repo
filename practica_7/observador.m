clear all;
T = 0.02;
s = tf('s')
P = 127.5/(s^2+10.02*s+128.6);
polos = pole(P);

%% Observador de tita y w
A = [0 1; -128.6 -10.02];
B = [0; 127.5];   C = [1 0];   D = 0;

% Verificar observabilidad (rango 2)
disp(rank(obsv(A, C)));

% Discretizacion
I = eye(2);
A_d = I + A*T;
B_d = B*T;
C_d = C;
D_d = D;

% Polos del observador: k veces más rápidos que los de la planta
k = 2;
p_d = exp(polos*k*T); % vector con ambos polos conjugados
abs(p_d); % deben ser < 1

L_d = place(A_d', C_d', p_d)'
eig(A_d - L_d*C_d); % verificar 

%% Observador extendido (tita, w y bias)
A = [0 1 0; -128.6 -10.02 0;0 0 0];
B = [0; 127.5; 0];   C = [1 0 0;0 1 1];   D = 0;

% Discretizacion
I = eye(3);
A_d = I + A*T;
B_d = B*T;
C_d = C;
D_d = D;

% Polos del observador: k veces más rápidos que los de la planta
k  = 5;
pb = -1; % polo del sesgo [rad/s]
p_d = [exp(polos(1)*k*T); exp(polos(2)*k*T); exp(pb*T)];
   
L_d = place(A_d', C_d', p_d)'
eig(A_d - L_d*C_d);
   


