%% PARAMETROS GENERALES
Ts = 0.02;
t = (0:Ts:55)';

referencia = zeros(size(t));
%% DATOS DE LA PLANTA SIN RETARDO

s = tf('s');

% Modelo del servo
P_s = 127.5/(s^2 + 10.02*s + 128.6);

% Modelo del carrito
P_c = -7.7023/(s*(s + 0.5)); %0.6552

% Planta completa: servo + conversion grados a radianes + carrito
P = minreal(P_s*(pi/180)*P_c);
Pd = c2d(P, Ts, 'zoh');

%% DATOS DE LA PLANTA CON RETARDO

s = tf('s');

% Modelo del servo
P_s = 127.5/(s^2 + 10.02*s + 128.6);

% Modelo del carrito
P_c = -7.7023/(s*(s + 0.6552));

% Retardo de 0.02 s aproximado mediante Padé de primer orden
D_pade = (1 - 0.01*s)/(1 + 0.01*s);

% Planta completa: servo + conversión grados a radianes + carrito + retardo
P_retardo = minreal(P_s*(pi/180)*P_c*D_pade);
%% ============================================================
%  PERTURBACIONES
%  ============================================================
d_impulso = zeros(size(t));
d_impulso(t == 5) = 1/Ts;

% Para simular un escalon en lugar del impulso
% d_escalon = zeros(size(t));
% d_escalon(t >= 5) = 1;

%% ============================================================
%  SECCION 1: PLANTA SIN RETARDO
%  ============================================================

%% CONTROLADOR P - SIN RETARDO
Kp_P = -0.9;

Tr_P = feedback(Kp_P*P_c, 1);
Gd_P = minreal(P_c/(1 + Kp_P*P_c));

[~, MF_P, ~, ~] = margin(Kp_P*P_c);
fprintf('MF P sin retardo: %.2f grados\n', MF_P);

% SIMULACION DEL IMPULSO
y_P = lsim(Tr_P, referencia, t) + lsim(Gd_P, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_P = lsim(Tr_P, referencia, t) + lsim(Gd_P, d_escalon, t);

u_P = Kp_P*(referencia - y_P);


%% CONTROLADOR PI - SIN RETARDO
Kp_PI = -1.2;
Ki_PI = -0.09;

C_PI = pid(Kp_PI, Ki_PI);
Tr_PI = feedback(C_PI*P_c, 1);
Gd_PI = minreal(P_c/(1 + C_PI*P_c));

[~, MF_PI, ~, ~] = margin(C_PI*P_c);
fprintf('MF PI sin retardo: %.2f grados\n', MF_PI);

% SIMULACION DEL IMPULSO
y_PI = lsim(Tr_PI, referencia, t) + lsim(Gd_PI, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_PI = lsim(Tr_PI, referencia, t)  + lsim(Gd_PI, d_escalon, t);

e_PI = referencia - y_PI;
u_PI = lsim(C_PI, e_PI, t);


%% CONTROLADOR PD - SIN RETARDO
Kp_PD = -1.2;
Kd_PD = -0.008;

C_PD = pid(Kp_PD, 0, Kd_PD);
Tr_PD = feedback(C_PD*P_c, 1);
Gd_PD = minreal(P_c/(1 + C_PD*P_c));

[~, MF_PD, ~, ~] = margin(C_PD*P_c);
fprintf('MF PD sin retardo: %.2f grados\n', MF_PD);

% SIMULACION DEL IMPULSO
y_PD = lsim(Tr_PD, referencia, t) + lsim(Gd_PD, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_PD = lsim(Tr_PD, referencia, t) + lsim(Gd_PD, d_escalon, t);

e_PD = referencia - y_PD;
u_PD = Kp_PD*e_PD + Kd_PD*[0; diff(e_PD)/Ts];


%% ============================================================
%  SECCION 2: PLANTA CON RETARDO (PADE)
%  ============================================================

D_pade = tf([-1 100], [1 100]);
P_retardo = minreal(P_c * D_pade);


%% CONTROLADOR P - CON RETARDO
Tr_P_ret = feedback(Kp_P*P_retardo, 1);
Gd_P_ret = minreal(P_retardo/(1 + Kp_P*P_retardo));

[~, MF_P_ret, ~, ~] = margin(Kp_P*P_retardo);
fprintf('MF P con retardo: %.2f grados\n', MF_P_ret);

% SIMULACION DEL IMPULSO
y_P_ret = lsim(Tr_P_ret, referencia, t) + lsim(Gd_P_ret, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_P_ret = lsim(Tr_P_ret, referencia, t) + lsim(Gd_P_ret, d_escalon, t);

u_P_ret = Kp_P*(referencia - y_P_ret);


%% CONTROLADOR PI - CON RETARDO
Tr_PI_ret = feedback(C_PI*P_retardo, 1);
Gd_PI_ret = minreal(P_retardo/(1 + C_PI*P_retardo));

[~, MF_PI_ret, ~, ~] = margin(C_PI*P_retardo);
fprintf('MF PI con retardo: %.2f grados\n', MF_PI_ret);

% SIMULACION DEL IMPULSO
y_PI_ret = lsim(Tr_PI_ret, referencia, t) + lsim(Gd_PI_ret, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_PI_ret = lsim(Tr_PI_ret, referencia, t)  + lsim(Gd_PI_ret, d_escalon, t);

e_PI_ret = referencia - y_PI_ret;
u_PI_ret = lsim(C_PI, e_PI_ret, t);


%% CONTROLADOR PD - CON RETARDO
Tr_PD_ret = feedback(C_PD*P_retardo, 1);
Gd_PD_ret = minreal(P_retardo/(1 + C_PD*P_retardo));

[~, MF_PD_ret, ~, ~] = margin(C_PD*P_retardo);
fprintf('MF PD con retardo: %.2f grados\n', MF_PD_ret);

% SIMULACION DEL IMPULSO
y_PD_ret = lsim(Tr_PD_ret, referencia, t)  + lsim(Gd_PD_ret, d_impulso, t);

% Para simular escalon, reemplazar d_impulso por d_escalon
% y_PD_ret = lsim(Tr_PD_ret, referencia, t)  + lsim(Gd_PD_ret, d_escalon, t);

e_PD_ret = referencia - y_PD_ret;
u_PD_ret = Kp_PD*e_PD_ret + Kd_PD*[0; diff(e_PD_ret)/Ts];

%% ============================================================
%  DATOS REALES
%  ============================================================

t_datos = out.tout(:);
distancia_datos = out.dist_carro(:);
control_datos = out.u(:);

idx = t_datos >= 0;
t_real = t_datos(idx);
y_real = distancia_datos(idx);
u_real = control_datos(idx);


%% ============================================================
%  GRAFICOS: PLANTA SIN RETARDO
%  ============================================================

figure;
subplot(2,1,1);
plot(t, y_P, 'b', t, y_PI, 'r', t, y_PD, 'g', t_real, y_real, 'k');
hold on;
yline(0, 'k:');
grid on;
title('Planta sin retardo - Posición del carro');
xlabel('Tiempo (s)');
ylabel('Posición (cm)');
legend('P', 'PI', 'PD', 'Datos reales', 'Referencia');

subplot(2,1,2);
plot(t, u_P, 'b', t, u_PI, 'r', t)