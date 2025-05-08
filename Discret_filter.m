%% Filtro Discreto
clear all;
close all;
clc;

Bel = [zeros(10,1); 1; zeros(9,1)]; %robot en celda 10
%Ejecuto los comandos de avanzar 9 y retroceder 3
for i = 1:9
    Bel = Discrete_Bayes_Filter(Bel, 'avanzar');
end
for i = 1:3
    Bel = Discrete_Bayes_Filter(Bel, 'retroceder');
end

%Grafico el belief final
bar(0:19, Bel, 'FaceColor', [0.2 0.6 0.8]);
xlabel('Celda');
ylabel('Probabilidad');
title('Belief de la posición del robot después de los movimientos');
grid on;

function new_Bel = Discrete_Bayes_Filter(Bel, command)
    N = length(Bel);
    new_Bel = zeros(N, 1);
    for i = 1:N
        if strcmp(command, 'avanzar')
            if i >= 3 %Aseguro que no se vaya del mundo
                new_Bel(i) = new_Bel(i) + 0.25*Bel(i-2);%Se mueve dos celdas el 25% de las veces 
            end
            if i >= 2 %Aseguro que no se vaya del mundo 
                new_Bel(i) = new_Bel(i) + 0.5*Bel(i-1);%Se mueve una celda el 50% de las veces
            end
            new_Bel(i) = new_Bel(i) + 0.25*Bel(i); %No se mueve de celda el 25% de las veces
        elseif strcmp(command, 'retroceder')
            if i <= N-2 %Aseguro que no se vaya del mundo
                new_Bel(i) = new_Bel(i) + 0.25*Bel(i+2);%Se mueve 2 celdas el 25% de las veces
            end
            if i <= N-1 %Aseguro que no se vaya del mundo
                new_Bel(i) = new_Bel(i) + 0.5*Bel(i+1);%Se mueve una celda el 50% de las veces
            end
            new_Bel(i) = new_Bel(i) + 0.25*Bel(i); %No se mueve el 25% de las veces
        end
    end    
    if strcmp(command, 'avanzar') %Defino limites en las ultimas celdas
        new_Bel(N) = Bel(N-1)*0.75 + Bel(N)*1;
        new_Bel(N-1) = new_Bel(N-1) - 0.25*Bel(N-1);
    elseif strcmp(command, 'retroceder') %Defino limites en las primeras celdas
        new_Bel(1) = 0.75*Bel(2) + 1*Bel(1);
        new_Bel(2) = new_Bel(2) - 0.25*Bel(2);
    end
end