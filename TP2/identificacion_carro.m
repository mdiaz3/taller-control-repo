%%
inicio=3.3;
fin=4.6;
Ts = 0.02; % periodo de muestreo

pos1 = out.distancia(1:end); % posicion del carro
tita1 = out.tita(1:end); % angulo de la barra

pos = [pos1];
tita = [tita1];

N = length(tita); % cant de muestras

% z ~ X * alpha
% z[k] = ( p[k+2] - 2*p[k+1] + p[k] )/ T^2
% X = tita[k]   -( p[k+1] - p[k] )/T
% alpha es el vector de coef en z[k]=c1​*tita[k]-c2​*(p[k+2]-p[k])/T​ 

X = [tita(1:N-2), -(pos(2:N-1) - pos(1:N-2))/Ts];
z = (pos(3:N) - 2*pos(2:N-1) + pos(1:N-2))/(Ts^2);
alpha = pinv(X)*z;

% coeficientes que minimizan la norma de z-X*alpha
g = alpha(1); a = alpha(2); % a=k/m

% funcion de transferencia continua
s = tf('s');
P_c = zpk(g/(s*(s+a))); 
polos_c = pole(P_c); % polos continuos

% funcion de transferencia discreta
P_d = c2d(P_c,Ts);
polos_d = pole(P_d); % polos discretos

% Resultados
P_c
datos_oscilacion=[tita,pos];
%% Grafico de los datos usados en la identificacion
t0 = 4;                                    % inicio del tramo [s]
t  = (0:N-1)' * Ts;
pos  = pos(:);  tita = tita(:);

figure;
subplot(2,1,1); plot(t0+t, tita); grid on;
ylabel('\theta [rad]'); title('Datos de identificacion');
subplot(2,1,2); plot(t0+t, pos); grid on;
ylabel('p [m]'); xlabel('Tiempo [s]');

%% Simulacion con la funcion de transferencia identificada
k  = 10;                                   % muestras para estimar condiciones iniciales
c  = polyfit(t(1:k), pos(1:k), 1);
x0 = [pos(1); c(1)];                       % [posicion inicial; velocidad inicial]

tita0 = 0;                                 % offset del angulo (0 si no se conoce)
sys = ss([0 1; 0 -a], [0; g], [1 0], 0);   % es P_c = g/(s(s+a)) con estados [pos; vel]
pos_sim = lsim(sys, tita - tita0, t, x0);

fit = 100*(1 - norm(pos - pos_sim)/norm(pos - mean(pos)));

figure;
plot(t0+t, pos, t0+t, pos_sim); grid on;
legend('medido', 'modelo');
xlabel('Tiempo [s]'); ylabel('p [m]');
title(sprintf('Ajuste: %.1f %%', fit));
