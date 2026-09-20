%%
u = out.u; % senial de control
y = out.alpha_f; % angulo estimado
N = length(y); % cant de muestras
Ts = 0.02; % periodo de muestreo

% y ~ X * alpha
% alpha es el vector de coef en y[k]=c1​y[k−1]+c2​y[k−2]+c3​u[k−2]​ 

X = [y(2:N-1), y(1:N-2), u(1:N-2)];r
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
P_c = d2c(P_d);
polos_c = pole(P_c); % polos continuos


%% Resultados
s = tf('s');
P_c = (-0.8081*s + 78.02)/(s^2 + 9.78*s + 90.37);

%% Graficar resultados
t = (0:N-1)' * Ts;
y_modelo = lsim(P_d, u, t);

plot(t, y, '.', t, y_modelo, '-');
legend('Datos', 'Modelo identificado');
xlabel('Tiempo [s]');
ylabel('\alpha_f');
grid on;