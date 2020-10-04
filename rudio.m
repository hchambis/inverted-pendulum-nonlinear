function rudio()
t0=0; 
tf=10; 
h=0.001;
% n veces que llama a la funcion nolineal para cada punto de solucion
N=4;
% FUERZA
UU=0;
% PARAMETROS
m=0.1;
M=2;
g=9.81;
l=1;
%% CONTROL LQR
A=[0 1 0 0;g*(M+m)/(M*l) 0 0 0;0 0 0 1;-g*m/M 0 0 0 ];
B=[0;-1/(M*l);0;1/M];
Q=[ 1 0 0 0; %penalizar y1
 0 0 0 0; %penalizar y2
 0 0 1 0; %penalizar y3
 0 0 0 0];%penalizar y4
R=1; %penalizar esfuerzo de actuador
##K=[-20.1877 -3.3678 -1.0000 -1.5924];
K=[-739.1273 -194.9463 -326.5473 -146.9463];
%% CONDICIONES INICIALES
y0 = [pi 0.5 0 0];
[t,ydo,d0]=RK4(@(t,y) nolineal(t,y),t0,tf,y0,h);
function ydot= nolineal(t,y)
% CONTROL LQR
u=-K*y;
% FUNCION LINEAL
ydot=[y(2);
 g*(M+m)/(M*l)*y(1)-1/(M*l)*u;
 y(4);
-g*m/M*y(1)+u/M];
 if(N==4)
 N=0;
 UU(end+1)=u; %guardamos cada valor de U
 end
 N=N+1;
endfunction
%% FUNCION RUNGE-KUTTA ORDEN 4
function [t,ydo,d0]=RK4(f,t0,tf,y0,h)
t = t0:h:tf;
y=y0';
ydo=y0;
%% FUNCION DE RUIDO
d0=0.005*randn(1,length(t));
d0(round(length(t)/2))=0.5; 
for i=1:(length(t)-1)
 k1 = f(t(i),y);
 k2 = f(t(i)+0.5*h,y+0.5*h*k1);
 k3 = f((t(i)+0.5*h),(y+0.5*h*k2));
 k4 = f((t(i)+h),(y+k3*h));
 y = y + (1/6)*(k1+2*k2+2*k3+k4)*h;
% SEÑALES MAS RUIDO
 y(1)=y(1)+d0(i);
 y(3)=y(3)+d0(i);
 ydo(i+1,:)=y;
end
endfunction
figure
subplot(2,2,1),
plot(t,ydo(:,1));
title('Ángulo');
xlabel('t'),ylabel('Amplitud');
grid
subplot(2,2,2),
plot(t,ydo(:,3));
title('Desplazamiento');
xlabel('t'),ylabel('Amplitud');
grid
subplot(2,2,3),
plot(t,d0);
title('Señal disturbio');
xlabel('t'),ylabel('Amplitud');
grid
subplot(2,2,4),
plot(t,UU);
title('Fuerza aplicada');
xlabel('t'),ylabel('Amplitud');
grid
endfunction
