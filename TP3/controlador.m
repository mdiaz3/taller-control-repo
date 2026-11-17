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
p_ctl_c = [-2+2j, -2-2j, -6, -7];    % punto de partida, a ajustar
p_ctl_d = exp(p_ctl_c*Ts);
K = place(A_d, B_d, p_ctl_d);        % 1x4
disp(K)
disp(abs(eig(A_d - B_d*K))')         % todos < 1

%% Simulación: perturbación tipo impulso en el carro
% (el carro recibe un golpe: una velocidad inicial repentina)
x = [0; 0.3; 0; 0];                  % ṗ = 0.3 m/s de golpe, ajustá hasta ver p_max ~ 0.1 m
N = 300;  X = zeros(4,N);  U = zeros(1,N);
for k = 1:N
    X(:,k) = x;
    U(k) = -K*x;
    x = A_d*x + B_d*U(k);
end
t = (0:N-1)*Ts;
figure; subplot(2,1,1); plot(t, X(1,:), t, X(3,:)); legend('p [m]','\theta [°]'); grid on
subplot(2,1,2); plot(t, U); ylabel('u [°]'); grid on