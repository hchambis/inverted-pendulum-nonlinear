%% FUNCION RUNGE-KUTTA ORDEN 4
function [t,ydo]=RK4(f,t0,tf,y0,h)
t = t0:h:tf;
length(t);
y=y0';
ydo=y0;
for i=1:(length(t)-1)
k1 = f(t(i),y);
k2 = f(t(i)+0.5*h,y+0.5*h*k1);
k3 = f((t(i)+0.5*h),(y+0.5*h*k2));
k4 = f((t(i)+h),(y+k3*h));
y = y + (1/6)*(k1+2*k2+2*k3+k4)*h;
ydo(i+1,:)=y;
end
end