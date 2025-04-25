%% Sensado
clc;
clear all;
close all;
%Posicion robot respecto terna global
p_rg = [5 -7 -pi/4];
%Sensor LIDAR posicion respecto el robot
p_lr = [0.2 0 pi];

%Matriz transformacion respecto terna robot al global 
T_rg = [cos(p_rg(3)) -sin(p_rg(3)) p_rg(1);
     sin(p_rg(3)) cos(p_rg(3)) p_rg(2);
     0 0 1];

%Matriz transformacion respecto a la terna de LIDAR a la del robot
T_lr = [cos(p_lr(3)) -sin(p_lr(3)) p_lr(1);
     sin(p_lr(3)) cos(p_lr(3)) p_lr(2);
     0 0 1];
 
%Matriz transformacion lidar-global
T_lg = T_rg*T_lr;

%Escaneo el espacio y grafico desde la terna global
scan = load('-ascii', 'laserscan.dat');
angle = linspace(-pi/2, pi/2, size(scan,2));

% Coordenadas en la terna del LIDAR
x_lidar = scan .* cos(angle);
y_lidar = scan .* sin(angle);

%Coordenadas desde la terna global
p_lidar = [x_lidar; y_lidar; ones(1, length(x_lidar))];
p_lidar_global = T_lg*p_lidar;
pi_lidar = T_lg.*p_lr; %Posicion LIDAR
% Graficar en la terna LIDAR
figure;
hold on;
plot(p_lidar(1,:), p_lidar(2,:), '*');
xlabel('x (LIDAR)');
ylabel('y (LIDAR)');
title('Mediciones LIDAR respecto terna LIDAR');
axis('equal');
grid on;
hold off;

% Graficar en la terna global
figure;
hold on;
plot(p_lidar_global(1,:), p_lidar_global(2,:), '*');
xlabel('x (Global)');
ylabel('y (Gloal)');
title('Mediciones LIDAR respecto terna global');
axis('equal');
grid on;
hold off;


%% Accionamiento diferencial 
clc;
clear all;
close all;
  
p_i = [0, 0, pi/4]; %[x y theta]
c = [0.1 0.5 2; %[v_l v_r t]
    0.5 0.1 2;
    0.2 0.2 2;
    1 0 4;
    0.4 0.4 2;
    0.2 -0.2 2;
    0.5 0.5 2];
l_r = 0.5;
N = size(c, 1);
mapeo = zeros(N + 1, 3);
mapeo(1, :) = p_i;
for i = 1:N
    [x_n, y_n, theta_n] = diffdrive(p_i(1), p_i(2), p_i(3), c(i,1), c(i, 2), c(i, 3), l_r);
    p_i = [x_n, y_n, theta_n];
    mapeo(i + 1, :) = p_i;
end
hold on;
plot(mapeo(:, 1), mapeo(:, 2), 'b');
quiver(mapeo(:, 1), mapeo(:, 2), cos(mapeo(:, 3)), sin(mapeo(:, 3)), 0.3, 'r', 'LineWidth', 1);
xlabel('X');
ylabel('Y');
title('Trayectoria robot con cinematica diferencial');
axis equal;
grid on;


function [x_n, y_n, theta_n] = diffdrive(x, y, theta, v_l, v_r, t, l)
   %Mapeo el movimiento en el marco de ref global al de ref local del robot
    T_gr = [cos(theta*t) -sin(theta*t) 0;
            sin(theta*t) cos(theta*t) 0;
            0 0 1];
        
    if abs(v_r - v_l) < 1e-6 %En caso de que las velocidades de las ruedas se parezcan
        x_n = x + (v_l + v_r)*t*cos(theta);
        y_n = y + (v_l + v_r)*t*sin(theta);
        theta_n = theta;
    else
        R = l*(v_l + v_r)/(2*(v_r - v_l)); 
        w = (v_r - v_l)/l;
        ICC = [(x - R*sin(theta)); (y + R*cos(theta)); 0];
        vec_pos = [x; y; theta];
        vec_rel = vec_pos - ICC;
        vec_rot = T_gr*vec_rel;
        result = vec_rot + ICC;
        x_n = result(1);
        y_n = result(2);
        theta_n = theta + w*t;
    end
end

