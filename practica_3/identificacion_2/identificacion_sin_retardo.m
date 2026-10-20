%% Identificacion planta del servo
%u = out.u(30/0.02:end); % senial de control
%y = out.alpha_f(30/0.02:end); % angulo estimado

u = datos(:,1);
y = datos(:,2);

N = length(y); % cant de muestras
Ts = 0.02; % periodo de muestreo

% y ~ X * alpha
% alpha es el vector de coef en y[k]=c1​y[k−1]+c2​y[k−2]+c3​u[k−2]​ 

X = [y(2:N-1), y(1:N-2), u(1:N-2)];
y_identificacion = y(3:N);
alpha = pinv(X)*y_identificacion;

% coeficientes que minimizan la norma de y-X*alpha
c1 = alpha(1);
c2 = alpha(2);
c3 = alpha(3);

% funcion de transferencia discreta
P_d = tf([0 0 c3],[1 -c1 -c2],Ts); 
polos_d = pole(P_d); % polos discretos

% funcion de transferencia continua
P_c = d2c(P_d,'matched')
polos_c = pole(P_c) % polos continuos

%% Resultados
s = tf('s');
P_c = 127.5/(s^2 + 10.02*s + 128.6)
pole(P_c);

% Con retardo
R = (2/Ts - s)/(2/Ts + s);
P_c_ret = minreal(P_c*R);


%% Graficar resultados
t = (0:N-1)' * Ts;
y_modelo = lsim(P_c, u, t);
y_modelo_ret = lsim(P_c_ret, u, t);

figure(); plot(t, y, '.', t, y_modelo, '-');
legend('Datos', 'Modelo identificado');
xlabel('Tiempo [s]');
ylabel('\tita_f');
grid on;