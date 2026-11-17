clear all;
s = tf('s');
a=3.28; g=-18.65*pi/180;Ts=0.02;

P_servo = 127.5/(s^2+10.02*s+128.6);
P_carrito = g/(s*(s+a));
P = P_servo * P_carrito;
polos = pole(P);

% Matrices continuas
A = [0 1 0 0; 0 -a g 0; 0 0 0 1; 0 0 -128.6 -10.02];  
B = [0;0;0;127.5];  
C = [1 0 0 0; 0 0 1 0];  
D = zeros(2,1);

% Discretizacion
sysd = c2d(ss(A,B,C,D), Ts, 'zoh');
[A_d,B_d,C_d,~] = ssdata(sysd);

%% Observador
rank(obsv(A,C)); % 8x4 > rango 4 > es observable

% si solo se mide la posicion:
rank(obsv(A,C(1,:))); % 4x4 > rango 4 > es observable

% si solo se mide el angulo
rank(obsv(A,C(2,:))); % 4x4 > rango 2 > no es observable 

% Polos del observador
p_obs_c = [-7 -8 -10 -11];          % ajustar manual
p_d = exp(p_obs_c*Ts);

L_d = place(A_d', C_d', p_d)';
eig(A_d - L_d*C_d); % verificar 

%% Impresión estilo Arduino
print_rows('a', A_d);
print_rows('b', B_d);
print_rows('l', L_d);

function print_rows(name, M)
    [r, c] = size(M);
    for i = 1:r
        items = cell(1, c);
        for j = 1:c
            if c == 1
                id = sprintf('%s%d', name, i);
            else
                id = sprintf('%s%d%d', name, i, j);
            end
            items{j} = sprintf('%s = %.6g', id, M(i,j));
        end
        fprintf('const float %s;\n', strjoin(items, ', '));
    end
    fprintf('\n');
end
                    
