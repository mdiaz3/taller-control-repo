%% Identificación del modelo físico del carrito

Ts = 0.02;  % Período de muestreo [s]

% Asegurar que sean vectores columna
tita = tita(:);  % Ángulo de la barra [rad]
pos = pos(:);    % Posición del carrito [m]

% Cantidad de muestras
N = min(length(tita), length(pos));

tita = tita(1:N);
pos = pos(1:N);

datos_oscilacion=[tita,pos];
%% 1. Estimación de derivadas

% Velocidad mediante diferencia centrada
vel = (pos(3:N) - pos(1:N-2))/(2*Ts);

% Aceleración mediante diferencia centrada
acel = (pos(3:N) - 2*pos(2:N-1) + pos(1:N-2))/(Ts^2);

% Ángulo correspondiente a las muestras centrales
theta = tita(2:N-1);

%% 2. Identificación por mínimos cuadrados

% Modelo físico:
% aceleracion = g*theta - a*velocidad
%
% A*alpha = b

A = [theta, -vel];
b = acel;

alpha = pinv(A)*b;

g_estimado = alpha(1);
a_estimado = alpha(2);

fprintf('g estimado = %.6f m/s^2\n', g_estimado);
fprintf('a estimado = %.6f 1/s\n', a_estimado);

%% 3. Función de transferencia continua

s = tf('s');

P_c = g_estimado/(s*(s + a_estimado));

disp('Transferencia continua del carrito:')
P_c

disp('Polos continuos:')
pole(P_c)

%% 4. Conversión a discreto

P_d = c2d(P_c, Ts, 'zoh');

disp('Transferencia discreta del carrito:')
P_d

disp('Polos discretos:')
pole(P_d)

%% 5. Comparación entre aceleración medida y estimada

acel_estimada = g_estimado*theta - a_estimado*vel;

t = (1:N-2)'*Ts;

figure
plot(t, acel, t, acel_estimada)
grid on
xlabel('Tiempo [s]')
ylabel('Aceleración [m/s^2]')
legend('Aceleración estimada de las mediciones', ...
       'Aceleración del modelo')
title('Validación del modelo físico del carrito')
