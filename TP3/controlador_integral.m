s = tf('s');
a=3.28; g=-18.65*pi/180;Ts=0.02;

P_servo = 127.5/(s^2+10.02*s+128.6);
P_carrito = g/(s*(s+a));
P = P_servo * P_carrito;
polos = pole(P);

% Matrices continuas aumentadas 
A = [0 1 0 0; 0 -a g 0; 0 0 0 1; 0 0 -128.6 -10.02];  
B = [0;0;0;127.5];  
C = [1 0 0 0; 0 0 1 0];  
D = zeros(2,1);

% Discretizacion
sysd = c2d(ss(A,B,C,D), Ts, 'zoh');
[A_d,B_d,C_d,~] = ssdata(sysd);

% Modelo aumentado: x_aug = [p; dp; tita; dtita; xi]
Cp = [1 0 0 0];                          % la salida que se integra: la posición
A_aug = [A_d, zeros(4,1); -Ts*Cp, 1];    % la última fila es xi[k+1] = xi[k] - Ts*p[k]
B_aug = [B_d; 0];                        % u no entra directamente en xi
B_ref = [zeros(4,1); Ts];                % p_ref entra solo por xi

rank(ctrb(A_aug, B_aug));             % debe dar 5

%% Matriz de ganancias
p_c = [-4+2j, -4-2j, -8, -9, -7];        % ajustar
K_aug = place(A_aug, B_aug, exp(p_c*Ts));
K  = K_aug(1:4)                         % ganancias de los 4 estados
Ki = K_aug(5)                          % ganancia del integrador
disp(K_aug);

%% Verificar especificaciones
K_aug_manual = [-200, -20, 0.9, 0.05, 150];
Acl = A_aug - B_aug*K_aug_manual;               % lazo cerrado (con estado real, sin ruido)
sys = ss(Acl, B_ref, [1 0 0 0 0], 0, Ts);% entrada: p_ref, salida: p
info = stepinfo(sys);                    % escalón unitario
fprintf('Ts = %.2f s, sobrepico = %.1f %%\n', info.SettlingTime, info.Overshoot);
impulse(sys)