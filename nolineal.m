%% FUNCION NO LINEAL, PÉNDULO INVERTIDO
function pendulo=nolineal(t,x)
% parámetros
m = 0.1;
M = 2;
L = 1;
g = -9.8;
k = 1;
c = 0;
I = 0;
% Fuerza aplicada
u = 0;
Sx = sin(x(1));
Cx = cos(x(1));
D = M*I+M*m*L*L+m*I+m*m*L*L-m*m*L*L*Cx*Cx;
N1 = I+m*L*L;
N2 = m+M;
% Sistema
pendulo =[x(2);
        (N2*m*L*g*Sx+N2*c*x(2)+m*L*k*x(4)*Cx-m*m*L*L*x(4)*x(4)*Sx*Cx-m*L*u*Cx)/(D);
  x(4);
        (N1*u-N1*k*x(4)+N1*m*L*x(2)*x(2)*Sx-m*m*L*L*g*Cx*Sx-m*L*c*x(2)*Cx)/(D)
    ];
end