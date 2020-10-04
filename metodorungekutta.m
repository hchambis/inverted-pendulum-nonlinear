%% SIMULACION USANDO RUNGE-KUTTA
t0=0; % start time
tf=100; % end time
h=0.1; %pasos
yo(1)= pi % initial condition angulo
yo(2)=0.5; % initial condition velocidad angular
yo(3)=0; % initial condition posicion
yo(4)=0; % initial condition velocidad
[t,ydo]=RK4(@(t,x) nolineal(t,x),t0,tf,yo,h);
figure()
plot(t,ydo),grid
title('Péndulo invertido con método Runge Kutta');
xlabel('t'),ylabel('Amplitud');
legend('Angulo','V. Angular','Desplazamiento','Velocidad');