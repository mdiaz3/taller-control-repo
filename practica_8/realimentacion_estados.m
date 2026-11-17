clear all;
Ts = 0.02;
A = [0 1; -128.6 -10.02];
B = [0; 127.5];   C = [1 0];   D = 0;

% Discretizacion
I = eye(2);
A_d = I + A*Ts;
B_d = B*Ts;
C_d = C;
D_d = D;

% Polos objetivo
zeda = 0.8; wn = 10; 
sigma = zeda*wn; 
wd = wn*sqrt(1-zeda^2);
s = -sigma + 1j*wd;
z = exp(s*Ts);
polos = [z conj(z)];

K = -place(A_d,B_d,polos)
F = 1/(C*inv(eye(2)-(A_d+B_d*K))*B_d)
