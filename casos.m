%% CASOS CON ODE45
tiempo = 0:.1:100;
% CASO I:
%y0 = [0; 0; 0; 0];
%y0 = [pi; 0; 0; 0];
% CASO II:
y0 = [pi; 0.5; 0; 0];
[t,y] = ode45(@(t,x) nolineal(t,x), tiempo, y0);
figure(1)
plot(t,y), grid on
title('Péndulo invertido con condiciones iniciales');
xlabel('t'),ylabel('Amplitud');
legend('Angulo','V. Angular','Desplazamiento','Velocidad');