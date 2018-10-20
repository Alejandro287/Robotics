%% Robot 3R
% Calculo del jabobiano para un manipulador 3R -> Jacobiano geometrico +
% Jacobiano analitico
clc 
clear
syms q1 q2 q3 L1 L2 L3

% MDHmod( alpha_i-1, a_i-1, d_i, theta_i)
TablaDH(1,:)= [    0,    0,    0,      q1];
TablaDH(2,:)= [    0,   L1,    0,      q2];
TablaDH(3,:)= [    0,   L2,    0,      q3];
TablaDH(4,:)= [    0,   L3,    0,       0];

% MTH de cada eslabon
A01 = MDHmod(TablaDH(1,:));
A12 = MDHmod(TablaDH(2,:));
A23 = MDHmod(TablaDH(3,:));
A34 = MDHmod(TablaDH(4,:));

%Cinematica Directa
A04 = simplify(A01*A12*A23*A34);
%% Jacobiano geometrico
% Calculo de los 0Zi 
z01 = A01(1:3,3); % Art 1
A02 = simplify(A01*A12); 
z02 = A02(1:3,3); % Art 2
A03 = simplify(A02*A23);
z03 = A03(1:3,3); % Art 3
%% Calculo del vector ^iP_n
p34 = A04(1:3,4) - A03(1:3,4);
p23 = A04(1:3,4) - A02(1:3,4);
p13 = A04(1:3,4) - A01(1:3,4);
%% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional 
J3 =simplify([(skew(z03) * p34);      z03]); % Rotacional 
% Tambien funciona cross(z03,p34)
Jgeo = simplify([J1, J2, J3]);

%% Jacobiano Simbolico
% Coordenadas generalizadas
pos_x = A04(1,4);
pos_y = A04(2,4);
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
l = [100 70 30];
q = [pi/6 pi/12 pi/24];
q_dot = [pi/3 pi/3 pi/6]';
%%
L1 = l(1);
L2 = l(2);
L3 = l(3);
q1 = q(1);
q2 = q(2);
q3 = q(3);
Jsym_0 = eval(Jsym);
Jgeo_0 = eval(Jgeo);
dot_x = Jsym_0*q_dot
%%
Jinv = simplify(inv(Jsym));
Jinv_0 = eval(Jinv)
Jinv_0*dot_x
q_dot
%%
% Singularidades
det(Jsym)













