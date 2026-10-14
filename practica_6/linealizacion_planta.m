clear variables; close all;
syms h n;
d=0.01065; g=9.81; I_1=0.1; I_2=0.4; L=0.9; 

x = [h]; % vector de estados
y = h; % salida

% valores de quilibrio
q_i = 8/60 * 1e-3;
h_eq = 0.45;
n_eq = q_i / (d^2*pi/4 * sqrt(2*g) * sqrt(h_eq));
x_eq = [h_eq];

% sistema no lineal
f1 = (q_i - d^2*pi/4 * n * sqrt(2*g) * sqrt(h))/(I_1 + (I_2-I_1)*h/L)^2;
f = [f1];

% linealizacion al rededor del quilibrio x_eq, u_eq
A = jacobian(f,x);
B = jacobian(f,n);
C = jacobian(y,x);
D = jacobian(y,n);

A1 = double(subs(A, {x(1), n}, {x_eq(1), n_eq}));
B1 = double(subs(B, {x(1), n}, {x_eq(1), n_eq}));
C1 = double(subs(C, {x(1), n}, {x_eq(1), n_eq}));
D1 = double(subs(D, {x(1), n}, {x_eq(1), n_eq}));

%% Planta linealizada
P = zpk(ss(A1,B1,C1,D1));

%% Controlador
close all;
s = tf('s');
P = zpk(-0.0042351/(s+0.00237));
C = zpk(-3*(s+0.00237)/ s);
Ts = 1;
C_d = c2d(C,Ts,'tustin')
L = minreal(P*C); % wgc = 0.0127
%figure(); margin(L); grid on;

T = feedback(L,1);
figure(); step(T);
