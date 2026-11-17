%% 1. PARAMETROS GENERALES

Ts = 0.02;
t = (0:Ts:55)';

referencia_cero = zeros(size(t));
referencia = 0.10*ones(size(t));

%% 2. PLANTA SIN RETARDO

s = tf('s');

P_s = 127.5/(s^2 + 10.02*s + 128.6);
P_c = -7.7023/(s*(s + 0.6552));

P = minreal(P_s*(pi/180)*P_c);

%% 3. PLANTA CON RETARDO

D_pade = (1 - 0.01*s)/(1 + 0.01*s);
P_retardo = minreal(P*D_pade);

%% 4. CONTROLADORES

% P
Kp_P = -0.9;
C_P = Kp_P;

% PI
Kp_PI = -1.75;
Ki_PI = -0.018;
C_PI = pid(Kp_PI, Ki_PI);

% PD
Kp_PD = -1.2;
Kd_PD = -0.008;
C_PD = pid(Kp_PD, 0, Kd_PD);

controladores = {C_P, C_PI, C_PD};
nombres = {'P', 'PI', 'PD'};

%% 5. PERTURBACION TIPO IMPULSO

d_impulso = zeros(size(t));
d_impulso(1) = 1/Ts;

%% 6. SIMULACIONES: UNA FIGURA POR CONTROLADOR

for i = 1:length(controladores)

    C = controladores{i};

    %% Transferencias de lazo cerrado

    % Seguimiento de referencia
    Tref_sin = feedback(C*P, 1);
    Tref_con = feedback(C*P_retardo, 1);

    % Perturbacion sumada a la entrada de la planta
    Tpert_sin = minreal(P/(1 + C*P));
    Tpert_con = minreal(P_retardo/(1 + C*P_retardo));

    %% Respuesta a perturbacion impulso

    y_imp_sin = lsim(Tpert_sin, d_impulso, t);
    y_imp_con = lsim(Tpert_con, d_impulso, t);

    % GUARDAR SIMULACION PROPORCIONAL SIN RETARDO
    if i == 1
        t_sim = t;
        pos_sim = y_imp_sin;
    end

    %% Respuesta a referencia escalon de 0.1 m

    y_ref_sin = lsim(Tref_sin, referencia, t);
    y_ref_con = lsim(Tref_con, referencia, t);

    %% Calculo de error estacionario aproximado

    ess_sin = referencia(end) - y_ref_sin(end);
    ess_con = referencia(end) - y_ref_con(end);

    fprintf('\n============================================\n');
    fprintf('CONTROLADOR %s\n', nombres{i});
    fprintf('============================================\n');

    fprintf('Estabilidad sin retardo: %s\n', ...
        string(isstable(Tref_sin)));
    fprintf('Estabilidad con retardo: %s\n', ...
        string(isstable(Tref_con)));

    fprintf('Error final sin retardo: %.6f m\n', ess_sin);
    fprintf('Error final con retardo: %.6f m\n', ess_con);

    %% FIGURA 1: PERTURBACION IMPULSO

    figure('Name', ['Perturbacion impulso - ' nombres{i}], ...
        'NumberTitle', 'off', ...
        'Units', 'pixels', ...
        'Position', [100 100 900 550], ...
        'PaperPositionMode', 'auto');

    plot(t, y_imp_sin, 'b', 'LineWidth', 1.4);
    hold on;
    plot(t, y_imp_con, 'r--', 'LineWidth', 1.4);
    yline(0, 'k:');

    grid on;
    xlabel('Tiempo [s]');
    ylabel('Posicion del carrito');
    title(['Respuesta al impulso - ' nombres{i}]);
    legend('Sin retardo', 'Con retardo', ...
        'Equilibrio', 'Location', 'best');

    %% FIGURA 2: SEGUIMIENTO DE REFERENCIA DE 0.1 m

    figure('Name', ['Referencia 0.1 m - ' nombres{i}], ...
        'NumberTitle', 'off', ...
        'Units', 'pixels', ...
        'Position', [100 100 900 550], ...
        'PaperPositionMode', 'auto');

    plot(t, referencia, 'k:', 'LineWidth', 1.6);
    hold on;
    plot(t, y_ref_sin, 'b', 'LineWidth', 1.4);
    plot(t, y_ref_con, 'r--', 'LineWidth', 1.4);

    grid on;
    xlabel('Tiempo [s]');
    ylabel('Posicion del carrito [m]');
    title(['Seguimiento de referencia - ' nombres{i}]);
    legend('Referencia 0.1 m', 'Sin retardo', ...
        'Con retardo', 'Location', 'best');

end

%% GUARDAR LAS SEIS FIGURAS

carpeta = 'Resultados_TP2';

if ~exist(carpeta, 'dir')
    mkdir(carpeta);
end

figuras = findall(0, 'Type', 'figure');

for k = 1:length(figuras)

    fig = figuras(k);
    nombre_archivo = regexprep(fig.Name, '[^\w.-]', '_');

    fig.Units = 'pixels';
    fig.Position = [100 100 900 550];

    exportgraphics(fig, ...
        fullfile(carpeta, [nombre_archivo '.png']), ...
        'Resolution', 300);
end

fprintf('\nSe guardaron %d figuras en: %s\n', ...
    length(figuras), fullfile(pwd, carpeta));

%% RESPUESTA AL IMPULSO: PD SIMULADA VS REAL

% Controlador PD (valores del Arduino)
Kp_PD = -1.2;
Kd_PD = -0.007;
C_PD = pid(Kp_PD, 0, Kd_PD);

% Planta sin retardo y transferencia ante perturbación
Tpert_PD = minreal(P/(1 + C_PD*P));

% Simulación ante perturbación impulso
t_sim_PD = t;
pos_sim_PD = lsim(Tpert_PD, d_impulso, t);

% Respuesta real del Arduino desde el segundo 13
t_datos = out.tout;
pos_datos = squeeze(out.dist_carro);

idx = t_datos >= 1;

t_real = t_datos(idx)-1;
pos_real = (pos_datos(idx) - 15)/100;

% Gráficos lado a lado
figure('Name', 'Simulación PD vs Arduino', ...
    'NumberTitle', 'off', ...
    'Position', [100 100 1200 500]);

% Simulación PD sin retardo
subplot(1,2,1);
plot(t_sim_PD, pos_sim_PD, 'b', 'LineWidth', 1.5);
hold on;
yline(0, 'k:');
grid on;
xlabel('Tiempo [s]');
ylabel('Desplazamiento [m]');
title('Simulación PD sin retardo');
legend('Simulación PD', 'Equilibrio', 'Location', 'best');

% Respuesta real del Arduino
subplot(1,2,2);
plot(t_real, pos_real, 'r', 'LineWidth', 1.2);
hold on;
yline(0, 'k:');
grid on;
xlabel('Tiempo [s]');
ylabel('Desplazamiento [m]');
title('Respuesta real del Arduino - PD');
legend('Arduino real', 'Equilibrio', 'Location', 'best');
xlim([0 6]);


%% SEGUIMIENTO DE REFERENCIA: P SIMULADO VS REAL

% Controlador proporcional
Kp_P = -0.9;
C_P = pid(Kp_P);

% Simulación del seguimiento de referencia
Tref_P = minreal((C_P*P)/(1 + C_P*P));
ref = -5*ones(size(t));  % Escalón unitario
pos_sim_P = lsim(Tref_P, ref, t);


% Gráficos separados
figure('Name', 'Seguimiento de referencia P', ...
    'NumberTitle', 'off', 'Position', [100 100 1200 500]);

% Seguimiento simulado
subplot(1,2,1);
plot(t, pos_sim_P, 'b', 'LineWidth', 1.5);
hold on;
yline(-5, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Salida');
title('Seguimiento de referencia P simulado');
legend('Salida simulada', 'Referencia', 'Location', 'best');


%% SEGUIMIENTO DE REFERENCIA: PI SIMULADO VS REAL

Kp_PI = -1.2;
Ki_PI = -0.09;
C_PI = pid(Kp_PI, Ki_PI);

% Simulación
Tref_PI = minreal((C_PI*P)/(1 + C_PI*P));
ref = -5*ones(size(t));
pos_sim_PI = lsim(Tref_PI, ref, t);

% Datos reales del Arduino desde el segundo 10
t_datos = out.tout;
pos_datos = squeeze(out.dist_carro);

idx = t_datos >= 0;
t_real = t_datos(idx) - 0;
pos_real = pos_datos(idx);

% Gráficos separados
figure('Name', 'Seguimiento de referencia PI', ...
    'NumberTitle', 'off', 'Position', [100 100 1200 500]);

subplot(1,2,1);
plot(t, pos_sim_PI, 'b', 'LineWidth', 1.5);
hold on;
yline(-5, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Salida');
title('Seguimiento PI simulado');
legend('Salida simulada', 'Referencia', 'Location', 'best');

subplot(1,2,2);
plot(t_real, pos_real, 'r', 'LineWidth', 1.2);
hold on;
yline(-5, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
title('Seguimiento PI real - Arduino');
legend('Arduino', 'Referencia', 'Location', 'best');
xlim([0 6]);


%% SEGUIMIENTO DE REFERENCIA: PD SIMULADO VS REAL

Kp_PD = -1.2;
Kd_PD = -0.008;
C_PD = pid(Kp_PD, 0, Kd_PD);

% Simulación
Tref_PD = minreal((C_PD*P)/(1 + C_PD*P));
ref = -5*ones(size(t));
pos_sim_PD = lsim(Tref_PD, ref, t);

% Datos reales del Arduino desde el segundo 10
t_datos = out.tout;
pos_datos = squeeze(out.dist_carro);

idx = t_datos >= 0;
t_real = t_datos(idx) - 0;
pos_real = pos_datos(idx);

% Gráficos separados
figure('Name', 'Seguimiento de referencia PD', ...
    'NumberTitle', 'off', 'Position', [100 100 1200 500]);

subplot(1,2,1);
plot(t, pos_sim_PD, 'b', 'LineWidth', 1.5);
hold on;
yline(-5, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Salida');
title('Seguimiento PD simulado');
legend('Salida simulada', 'Referencia', 'Location', 'best');

subplot(1,2,2);
plot(t_real, pos_real, 'r', 'LineWidth', 1.2);
hold on;
yline(-5, 'k--');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
title('Seguimiento PD real - Arduino');
legend('Arduino', 'Referencia', 'Location', 'best');
xlim([0 6]);