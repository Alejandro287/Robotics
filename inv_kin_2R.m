%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calcula la cinematica inversa para un mecanismo 2 R
% Entradas:
% x = Vector cartesiano de X
% y = Vector cartesiano de Y
% L1 =
% L2 =
% codo = Codo arriba (1), codo abajo (~1)
% Salidas:
% q = Valores angulares para las dos articulaciones
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q, check] = inv_kin_2R(x,y,L1,L2,codo)

num = x.^2+y.^2-L1^2-L2^2;
den = 2*L1*L2;
D = num./den;
check = (D<=1);

if check
    q2 = atan2(-sqrt(1-D.^2),D);
    
    art = codo;
    if art == 1
        q2 = atan2(sqrt(1-D.^2),D);
    end
    
    q1 = atan2(y,x) - atan2(L2*sin(q2), L1+L2*cos(q2));
    
    q = [q1; q2];
else
    %warning('No real solution')
    q = [NaN NaN];
end
