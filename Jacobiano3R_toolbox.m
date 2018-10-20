%%
% Toolbox
syms q1 q2 q3 L1 L2

L(1) = Link('revolute' ,'alpha',0,    'a',0, 'd',0     ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',L1, 'd',0     ,'offset',0, 'modified');
L(3) = Link('revolute' ,'alpha',0,    'a',L2, 'd',0     ,'offset',0, 'modified');
RRR = SerialLink(L,'name','3R');
%%
% Cinemática directa
T01 = simplify(L(1).A(q1));
T12 = simplify(L(2).A(q2));
T23 = simplify(L(3).A(0));
T02 = simplify(RRR.A([1 2],[q1 q2]));
T03 = simplify(RRR.A([1 2 3],[q1 q2 0]));
%% Calculo de los 0Zi
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
z03 = T03(1:3,3); % Art 3
%% Calculo del vector ^iP_n
p34 = T03(1:3,4) - T03(1:3,4);
p23 = T03(1:3,4) - T02(1:3,4);
p13 = T03(1:3,4) - T01(1:3,4);
%% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional 
J3 =simplify([(skew(z03) * p34);      z03]); % Rotacional 
% Tambien funciona cross(z03,p34)

Jgeo = simplify([J1, J2, J3]);
% Jacobiano reducido
Jgeo = [Jgeo(1:2,:);Jgeo(6,:)];

%% Jacobiano Simbolico
pos_x = T03(1,4);
pos_y = T03(2,4);
phi = q1 + q2 + q3;

%% Casillas del Jacobiano
J11 = diff(pos_x,q1);
J12 = diff(pos_x,q2);
J13 = diff(pos_x,q3);

J21 = diff(pos_y,q1);
J22 = diff(pos_y,q2);
J23 = diff(pos_y,q3);

J31 = diff(phi,q1);
J32 = diff(phi,q2);
J33 = diff(phi,q3);

Jsym = [J11 J12 J13;
        J21 J22 J23;
        J31 J32 J33];
%% 
% Ejercicio numerico
l = [100 70];
q = deg2rad([45 30 0]);
dot_x = [200 500 0]';
%%
L1 = l(1);
L2 = l(2);
q1 = q(1);
q2 = q(2);
q3 = q(3);
Jsym_0 = eval(Jsym);
Jgeo_0 = eval(Jgeo);
Jinv = simplify(inv(Jsym));
Jinv_0 = eval(Jinv);
disp(Jinv_0*dot_x)
%%
% Singularidades
det(Jsym)
