%%
y = out.d1;
u = out.d2;

u = u(:);
y = y(:);

N = length(y);

A = [y(2:N-1), y(1:N-2), u(1:N-2)];

b = y(3:N);

c = pinv(A)*b;

c1 = c(1);
c2 = c(2);
c3 = c(3);

fprintf('c1 = %.6f\n', c1);
fprintf('c2 = %.6f\n', c2);
fprintf('c3 = %.6f\n', c3);

num_z = [0 0 c3];
den_z = [1 -c1 -c2];

Ts = 0.02;     % 50 Hz

sys_d = tf(num_z, den_z, Ts);

disp('Sistema discreto:');
sys_d;



%% Pasar a continuo

sys_c = d2c(sys_d);

disp('Sistema continuo:');
sys_c

disp('Polos:');
pole(sys_c)
%% Graficar resultados
plot(out.d2, out.d1, 'o', out.d2, y_fit, '-');
legend('Datos', 'Ajuste lineal');