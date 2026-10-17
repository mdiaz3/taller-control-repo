%% Datos (pos en m, tita en rad)
close all
N  = length(tita);                      % cant de muestras
Ts = 0.02;                              % periodo de muestreo
t  = (0:N-1)' * Ts;

%Identificacion (forma integral)
% pos(t) = c0 + c1*t + c2*t^2 + g*II(tita) - a*I(pos)
I1 = cumtrapz(t, pos);
I2 = cumtrapz(t, cumtrapz(t, tita));
X  = [ones(N,1), t, t.^2, I2, -I1];
alpha = X \ pos;

g = alpha(4);  a = alpha(5);            % a = k/m

% funcion de transferencia continua
s = tf('s');
P_c = zpk(g/(s*(s+a)));
polos_c = pole(P_c);

% funcion de transferencia discreta
P_d = c2d(P_c, Ts);
polos_d = pole(P_d);

% Validacion: simular con el angulo medido y comparar con la posicion real
tita0 = -2*alpha(3)/g;   
x0 = [alpha(1); alpha(2) - a*alpha(1)];
sys = ss([0 1; 0 -a], [0; g], [1 0], 0);
pos_sim = lsim(sys, tita - tita0, t, x0);

fit = 100*(1 - norm(pos - pos_sim)/norm(pos - mean(pos)));  % 100 = ajuste perfecto
plot(t, pos, t, pos_sim);
legend('medido','modelo'); xlabel('Tiempo [s]'); ylabel('Posicion [m]'); grid on;
title(sprintf('Ajuste: %.1f %%', fit));

P_c


%% Cargar las corridas y calcular aceleracion vs angulo medio
archivos = {'experimento1','experimento2','experimento3','experimento4','experimento5','experimento6','experimento7','experimento8','experimento8'};  % ajustar
Ts = 0.02;   L = 40;                         % ventana de ~0.8 s
nrun = numel(archivos);
A = zeros(nrun,1);  thm = zeros(nrun,1);  ths = zeros(nrun,1);
i0v = zeros(nrun,1);  rp = zeros(nrun,1);  rt = zeros(nrun,1);

for j = 1:nrun
    S  = load(archivos{j});                  % queda en S.pos y S.tita
    p  = S.pos(:);   th = S.tita(:);
    rp(j) = max(p)  - min(p);                % rango de la corrida completa (control de columnas)
    rt(j) = max(th) - min(th);

    % inicio del movimiento: primera muestra donde pos se aparta 1 cm del reposo
    k = find(abs(p - p(1)) > 0.01, 1);
    if isempty(k), i0 = 1; else, i0 = max(1, k - 5); end
    i0v(j) = i0;
    idx = i0:min(i0+L-1, numel(p));
    if numel(idx) < L, warning('Corrida %d: solo %d muestras libres', j, numel(idx)); end

    pw = p(idx);   tw = th(idx);
    t  = (0:numel(idx)-1)' * Ts;
    c  = polyfit(t, pw, 2);                  % pos ~ p0 + v0*t + A*t^2/2
    A(j)   = 2*c(1);                         % aceleracion [m/s^2]
    thm(j) = mean(tw);                       % angulo medio [rad]
    ths(j) = std(tw);                        % cuanto varia el angulo en la ventana
end

disp(table((1:nrun)', i0v, rp, rt, thm, ths, A, ...
    'VariableNames', {'corrida','inicio','rango_pos','rango_tita','tita_medio','tita_std','A'}))

q = polyfit(thm, A, 1);                      % A = g*thm - g*tita0
figure; plot(thm, A, 'o', thm, polyval(q, thm)); grid on;
xlabel('\theta medio [rad]'); ylabel('Aceleracion [m/s^2]');
title(sprintf('pendiente = %.2f, corte con A=0 en tita = %.3f rad', q(1), -q(2)/q(1)));


%% estimar a
g = q(1);   tita0 = -q(2)/q(1);
a_run = zeros(nrun,1);

for j = 1:nrun
    S = load(archivos{j});   p = S.pos(:);   th = S.tita(:);
    k  = find(abs(p - p(1)) > 0.01, 1);   i0 = max(1, k - 5);
    p  = p(i0:end);   th = th(i0:end);
    N  = numel(p);    t  = (0:N-1)' * Ts;

    % pos = p0 + (v0 + a*p0)*t + g*II(th - tita0) - a*I(pos)
    I1 = cumtrapz(t, p);
    I2 = cumtrapz(t, cumtrapz(t, th - tita0));
    X  = [ones(N,1), t, -I1];
    c  = X \ (p - g*I2);          % c = [p0; v0 + a*p0; a]
    a_run(j) = c(3);
end

disp(table((1:nrun)', a_run, 'VariableNames', {'corrida','a'}))
a = median(a_run)

%% Validacion de una corrida: modelo fijo vs datos
Ts = 0.02;   L = 40;
archivos = {'experimento1','experimento2','experimento3','experimento4','experimento5','experimento6','experimento7','experimento8','experimento8'};  % ajustar

% g y tita0 de la recta (si q esta en el workspace los usa; si no, valores a mano)
if exist('q','var')
    g = q(1);   tita0 = -q(2)/q(1);
else
    g = -6.75;  tita0 = -0.008;
end

a = 0.6552;
j = 9;        

S = load(archivos{j});   p = S.pos(:);   th = S.tita(:);
k = find(abs(p - p(1)) > 0.01, 1);
if isempty(k), i0 = 1; else, i0 = max(1, k - 5); end
p  = p(i0:end);   th = th(i0:end);
t  = (0:numel(p)-1)' * Ts;

Lj = min(L, numel(p));                       % evita pasarse si la corrida es corta
if Lj < L, fprintf('Corrida %d: solo %d muestras libres\n', j, numel(p)); end
c  = polyfit(t(1:Lj), p(1:Lj), 2);
x0 = [c(3); c(2)];                           % posicion y velocidad inicial

sys = ss([0 1; 0 -a], [0; g], [1 0], 0);     % P_c = g/(s(s+a)) con estados [pos; vel]
p_sim = lsim(sys, th - tita0, t, x0);

ajuste = 100*(1 - norm(p - p_sim)/norm(p - mean(p)));
figure; plot(t, p, t, p_sim); grid on;
legend('medido', 'modelo'); xlabel('Tiempo [s]'); ylabel('Posicion [m]');
title(sprintf('Corrida %d, a = %.2f: ajuste %.1f %%', j, a, ajuste));

%% Validacion con rozamiento Coulomb + viscoso (modelo no lineal), una corrida
Ts = 0.02;   L = 40;
archivos = {'experimento1','experimento2','experimento3','experimento4','experimento5'};  % ajustar

g     = -8.44;      % [m/s^2/rad] ganancia (ajuste con Coulomb; no usar el -6.75 de la recta)
tita0 = -0.0073;    % [rad]       offset del angulo
mu    = 0.224;      % [m/s^2]     rozamiento tipo Coulomb (poner 0 para desactivarlo)
a     = 0.5;          % [1/s]       rozamiento viscoso
eps_v = 0.02;       % [m/s]       suavizado de sign(v)
j     = 3;          % <-- corrida a validar

S = load(archivos{j});   p = S.pos(:);   th = S.tita(:);
k = find(abs(p - p(1)) > 0.01, 1);
if isempty(k), i0 = 1; else, i0 = max(1, k - 5); end
p  = p(i0:end);   th = th(i0:end);
t  = (0:numel(p)-1)' * Ts;

Lj = min(L, numel(p));
if Lj < L, fprintf('Corrida %d: solo %d muestras libres\n', j, numel(p)); end
c  = polyfit(t(1:Lj), p(1:Lj), 2);
x0 = [c(3); c(2)];                           % posicion y velocidad inicial

% p'' = g*(tita - tita0) - a*p' - mu*sign(p')   (con sign suavizado)
acc = @(tt, x) [x(2); ...
    g*(interp1(t, th, tt, 'linear', 'extrap') - tita0) - a*x(2) - mu*tanh(x(2)/eps_v)];
[~, X] = ode45(acc, t, x0);
p_sim = X(:,1);

ajuste = 100*(1 - norm(p - p_sim)/norm(p - mean(p)));
figure; plot(t, p, t, p_sim); grid on;
legend('medido', 'modelo'); xlabel('Tiempo [s]'); ylabel('Posicion [m]');
title(sprintf('Corrida %d, mu = %.2f, a = %.2f: ajuste %.1f %%', j, mu, a, ajuste));