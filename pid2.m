function pid2()
to=0; 
tf=10; 
T=0.001;
t=to:T:tf;
%% CONDICIONES INICIALES
y0 = [pi 0.5 0 0];
% parameters
m=0.1;
M=2;
g=9.81;
l=1;
%% CONTROLADORES PID
% ANGULO
ref_theta=0;
kp_theta=10;
kd_theta=10;
ki_theta=0;
err_pas_theta=0;
integral_theta=0;
%% POSICION
ref_pos=5;
kp_pos=0;
kd_pos=0;
ki_pos=0;
err_pas_pos=0;
integral_pos=0;
[t,ydot] = ode45(@(t,y)nolineal(t,y),t,y0);
figure
subplot(2,1,1)
plot(t,ydot(:,1))
title('Ángulo')
grid
subplot(2,1,2)
plot(t,ydot(:,3))
title('Posicion ')
grid
function u=fuerza(t,y)
salida_pos=y(3);
err_pos=ref_pos-salida_pos;
proporcional=err_pos;
integral_pos=integral_pos+(T/2)*(err_pos+err_pas_pos);
derivativo=(err_pos-err_pas_pos)/T;
pid_pos= kp_pos*proporcional +ki_pos*integral_pos ...
+kd_pos*derivativo;
err_pas_pos=err_pos;
variacion_theta=pid_pos;
salida_theta=y(1);
err_theta=ref_theta-salida_theta+variacion_theta;
proporcional=err_theta;
integral_theta=integral_theta+(T/2)*(err_theta+err_pas_theta);
derivativo=(err_theta-err_pas_theta)/T;
pid_theta= kp_theta*proporcional +ki_theta*integral_theta ...
+kd_theta*derivativo;
err_pas_theta=err_theta;
u=-pid_theta;
end
% PENDULO INVERTIDO LINEALIZADO
function ydot=nolineal(t,y)
% parámetros
m = 0.1;
M = 2;
L = 1;
g = 9.8;
k = 1;
c = 0;
I = 0;
% Fuerza aplicada
%u = 0;
u=fuerza(t,y);
Sx = sin(y(1));
Cx = cos(y(1));
D = M*I+M*m*L*L+m*I+m*m*L*L-m*m*L*L*Cx*Cx;
N1 = I+m*L*L;
N2 = m+M;
% Sistema
ydot =[y(2);
        (N2*m*L*g*Sx+N2*c*y(2)+m*L*k*y(4)*Cx-m*m*L*L*y(4)*y(4)*Sx*Cx-m*L*u*Cx)/(D);
  y(4);
        (N1*u-N1*k*y(4)+N1*m*L*y(2)*y(2)*Sx-m*m*L*L*g*Cx*Sx-m*L*c*y(2)*Cx)/(D)
    ];
end
end