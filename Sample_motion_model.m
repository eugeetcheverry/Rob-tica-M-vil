%% Modelo de movimiento basado en odometria
clear all;
close all;
clc;

xt = [2.0; 4.0; pi/2];
ut = [pi/4; 0.0; 1.0];
alpha = [0.1; 0.1; 0.01; 0.01];
N = 5000; %Nro de muestras 
muestras = zeros(N, 2);

for i = 1:N
    pos = motion_model_odometry(xt, ut, alpha);
    muestras(i, :) = pos(1:2);
end

% Gráfico
figure;
plot(muestras(:,1), muestras(:,2), '.');
xlabel('x_{t+1}');
ylabel('y_{t+1}');
title('Muestras del modelo de movimiento con Odometría');
axis equal;
grid on;

function posicion = motion_model_odometry(xt, ut, alpha)
     %Genero el ruido utilizando las funciones del inciso anterior
    d_rot1 = ut(1) + BoxMuller(0, 1, 1)*(alpha(1)*abs(ut(1))+alpha(2)*abs(ut(3))); %Odometria con ruido
    d_rot2 = ut(2) + BoxMuller(0, 1, 1)*(alpha(1)*abs(ut(2))+alpha(2)*abs(ut(3))); %Odometria con ruido
    d_trans = ut(3) + BoxMuller(0, 1, 1)*(alpha(3)*abs(ut(3))+alpha(4)*(abs(ut(1))+ abs(ut(2)))); %Odomteria con ruido
    xt2 = xt(1) + d_trans.*cos(xt(3) + d_rot1); %Calculo x(t + 1)
    yt2 = xt(2) + d_trans.*sin(xt(3) + d_rot1); %Calculo y(t + 1)
    theta2 = xt(3) + d_rot1 + d_rot2; % Calculo theta(t + 1)
    posicion = [xt2 yt2 theta2];
end 

function muestras3 = BoxMuller(mu, sigma2, N)
    M = ceil(N/2);
    U1 = rand(M, 1); %Genero una unidorme entre 0 y 1
    U2 = rand(M, 1); % Genero una uniforme entre 0 y 1
    Z = cos(2*pi*U1)*sqrt(-2*log(U2));
    muestras3 = Z*sqrt(sigma2) + mu; %Escalo y traslado para centra la normal
end

