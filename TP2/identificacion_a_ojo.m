clear; close all; clc;

Ts = 0.02;
g = -8.5; a = 2.5;

A = [0 1; 0 -a];
B = [0; g];
C = [1 0];
D = 0;
P_c_ss = ss(A, B, C, D);

for i = 1:9
    load(sprintf('experimento%d.mat', i), 'pos', 'tita');

    N = length(tita);
    t = (0:N-1)' * Ts;

    pos_sim = lsim(P_c_ss, tita, t, [pos(1); 0]);

    figure();
    plot(t, pos*100, '-', t, pos_sim*100, '--');
    legend('Medido', 'Modelo');
    xlabel('Tiempo [s]');
    ylabel('Posición [cm]');
    title(sprintf('Experimento %d, tita = %.2f°', i, tita(5)*180/pi));
    grid on;
end