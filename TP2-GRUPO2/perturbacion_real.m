%% SEGUIMIENTO DE REFERENCIA: P SIMULADO VS REAL

% Controlador proporcional
Kp_P = -0.9;
C_P = pid(Kp_P);

% Simulación del seguimiento de referencia
Tref_P = minreal((C_P*P)/(1 + C_P*P));
ref = ones(size(t));  % Escalón unitario
pos_sim_P = lsim(Tref_P, ref, t);

% Datos reales del Arduino desde el segundo 10
t_datos = out.tout;
pos_datos = squeeze(out.dist_carro);

idx = t_datos >= 10;
t_real = t_datos(idx) - 10;
pos_real = pos_datos(idx);

% Gráficos separados
figure('Name', 'Seguimiento de referencia P', ...
    'NumberTitle', 'off', 'Position', [100 100 1200 500]);

% Seguimiento simulado
subplot(1,2,1);
plot(t, pos_sim_P, 'b', 'LineWidth', 1.5);
hold on;
yline(1, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Salida');
title('Seguimiento de referencia P simulado');
legend('Salida simulada', 'Referencia', 'Location', 'best');

% Seguimiento real del Arduino
subplot(1,2,2);
plot(t_real, pos_real, 'r', 'LineWidth', 1.2);
hold on;
yline(15, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
title('Seguimiento de referencia P real');
legend('Arduino', 'Referencia (15 cm)', 'Location', 'best');
xlim([0 12]);