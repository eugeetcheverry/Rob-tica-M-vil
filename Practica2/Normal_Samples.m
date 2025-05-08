%% Muestreo de Distribuciones de Probabilidad
clear all;
close all;
clc;

%Parámetros de la distribución
mu = 0;%Media
sigma2 = 1;%Varianza
N = 10000;%Nro de muestras

%Generar las muestras
sample1 = sample_normal_distribution(mu, sigma2, N);
sample2 = sample_distribution(mu, sigma2, N);
sample3 = BoxMuller(mu, sigma2, N);

%Graficar los histogramas
figure;

% Método de los 12 uniformes
subplot(3,1,1);
histogram(sample1, 'Normalization', 'pdf', 'BinWidth', 0.1);
hold on;
x = linspace(-5,5,1000);
plot(x, normpdf(x, mu, sqrt(sigma2)), 'r', 'LineWidth', 2);
title('Normal por suma de 12 Uniformes');
xlabel('x'); ylabel('Densidad');
legend('Histograma','Normal Teórica');
grid on;

% Método de aceptación-rechazo
subplot(3,1,2);
histogram(sample2, 'Normalization', 'pdf', 'BinWidth', 0.1);
hold on;
plot(x, normpdf(x, mu, sqrt(sigma2)), 'r', 'LineWidth', 2);
title('Normal por muestreo con rechazo');
xlabel('x'); ylabel('Densidad');
legend('Histograma','Normal Teórica');
grid on;

% Método de Box-Muller
subplot(3,1,3);
histogram(sample3, 'Normalization', 'pdf', 'BinWidth', 0.1);
hold on;
plot(x, normpdf(x, mu, sqrt(sigma2)), 'r', 'LineWidth', 2);
title('Normal por Box-Muller');
xlabel('x'); ylabel('Densidad');
legend('Histograma','Normal Teórica');
grid on;

function sample1 = sample_normal_distribution(mu, sigma2, N)
    U = rand(N, 12); %Genero muestras de uniformes
    sum_U = sum(U, 2); %Sumo las unformes
    sample_est = sum_U - 6; % Resto la media de la suma (6) para centrar en 0, obteniendo normal (0, 1)
    sample1 = sample_est*sqrt(sigma2) + mu; %Mediante un escalamiento y desplazamiento, obtengo la normal
end

function sample2 = sample_distribution(mu, sigma2, N)
    a = mu - 3*sigma2;%Defino limites [a; b]
    b = mu + 3*sigma2;
    f = @(x) (1/(sqrt(2*pi*sigma2)))*exp(-0.5*((x - mu).^2)/sigma2);%Comparo con la funcion de densidad
    i = 1;
    sample2 = zeros(1, N);
    while i < N
        u = rand();
        y = a + (b - a)*rand();
        if u < f(y)%Filtro las muestras
            sample2(i) = y;
            i = i+1;
        end
    end
end

function sample3 = BoxMuller(mu, sigma2, N)
    M = ceil(N/2);
    U1 = rand(M, 1); %Genero una unidorme entre 0 y 1
    U2 = rand(M, 1); % Genero una uniforme entre 0 y 1
    Z = cos(2*pi*U1).*sqrt(-2*log(U2));
    sample3 = Z*sqrt(sigma2) + mu; %Escalo y traslado para centra la normal
end
