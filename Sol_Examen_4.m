% Examen 4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% No es invertible, por ende el problema cinemático diferencial inverso se
% debe apoyar en la pseudoinversa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
syms q1 q2 q3 q4 
l = [1 1 1  0.5 0.5];

L(1) = Link('prismatic' ,'alpha',0,     'a',0,    'theta',0 ,'offset',0, 'qlim', [0 2], 'modified');
L(2) = Link('revolute'  ,'alpha',-pi/2, 'a',0,    'd',l(1)  ,'offset',0,                'modified');
L(3) = Link('prismatic' ,'alpha',0,     'a',l(2), 'theta',0 ,'offset',0, 'qlim', [0 2], 'modified');
L(4) = Link('revolute'  ,'alpha',-pi/2, 'a',0,    'd',l(3)  ,'offset',0,                'modified');
L(5) = Link('revolute'  ,'alpha',0,     'a',l(4), 'd',l(5)  ,'offset',0,                'modified');

P4 = SerialLink(L,'name','PRPR');
q = [0 0 0 0 0];

trplot(eye(4),'rgb','frame','0')
hold on;
P4.plot(q,'workspace',[-3 3 -3 3 -2 3],'scale',0.5,'tilesize',1,'floorlevel',-1)
axis([-3 3 -3 3 -2 3])
P4.teach()
M = eye(4);
for i=1:P4.n
    M = M * L(i).A(q(i));
    trplot(M,'rgb','frame',num2str(i),'length',1)
end
%% MTH
T01 = L(1).A(q1);
T12 = L(2).A(q2);
T12 = MTH_simplex(T12);
T12(2,1) = 0;
T12(2,2) = 0;
T02 = T01*T12;
T23 = L(3).A(q3);
T03 = T02*T23;
T34 = L(4).A(q4);
T34 = MTH_simplex(T34);
T34(2,1) = 0;
T34(2,2) = 0;
T04 = T03*T34;
T45 = L(5).A(0);
T05 = simplify(T04*T45);

%% Calculo de los 0Zi
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
z03 = T03(1:3,3); % Art 3
z04 = T04(1:3,3); % Art 3
z05 = T05(1:3,3); % Art 3
%% Calculo del vector ^iP_n
p45 = T05(1:3,4) - T04(1:3,4);
p35 = T05(1:3,4) - T03(1:3,4);
p25 = T05(1:3,4) - T02(1:3,4);
p15 = T05(1:3,4) - T01(1:3,4);
%% Calculo de las columnas del jabobiano
J1 =simplify([z01; [0 0 0]']); % Prismatico
J2 =simplify([(skew(z02) * p25); z02]); % Rotacional 
J3 =simplify([z03; [0 0 0]']); % Prismatico
J4 =simplify([(skew(z04) * p45); z04]); % Rotacional

Jgeo = simplify([J1 J2 J3 J4]);
%%
q1 = 2;
q2 = pi/12;
q3 = 0.5;
q4 = pi/4;
Jgeo_eval = eval(Jgeo)
disp('-------------------------------------------------')
P4.jacob0([2 pi/12 0.5 pi/4 0])
%%
J0_q0 = P4.jacob0(q);
J0_q0 = J0_q0(:,1:4);
q_dot = [1 0 1 0]';
x_dot = J0_q0*q_dot
%% Tercera art
art3 = SerialLink(L(1:3),'name','PRP');
art3.plot([0 0 0],'workspace',[-3 3 -3 3 -2 3],'scale',0.5,'tilesize',1,'floorlevel',-1)
axis([-3 3 -3 3 -2 3])
art3.teach()
%% Calculo del vector ^iP_n
p33 = T03(1:3,4) - T03(1:3,4);
p23 = T03(1:3,4) - T02(1:3,4);
p13 = T03(1:3,4) - T01(1:3,4);
%% Calculo de las columnas del jacobiano
J1_art3 =simplify([z01; [0 0 0]']); % Prismatico
J2_art3 =simplify([(skew(z02) * p23); z02]); % Rotacional 
J3_art3 =simplify([z03; [0 0 0]']); % Prismatico

Jgeo_art3 = simplify([J1_art3 J2_art3 J3_art3]);
%%
q1 = 2;
q2 = pi/12;
q3 = 0.5;
Jgeo_art3_eval = eval(Jgeo_art3)
disp('-------------------------------------------------')
art3.jacob0([2 pi/12 0.5])
%% Derivada Matriz de Rotación
syms q1 q2 q3 q4 real

dx = T05(1,4);  % componente x
dy = T05(2,4);  % componente y
dz = T05(3,4);  % componente z

vx = [diff(dx,q1), diff(dx,q2), diff(dx,q3), diff(dx,q4)]; 
vy = [diff(dy,q1), diff(dy,q2), diff(dy,q3), diff(dy,q4)];
vz = [diff(dz,q1), diff(dz,q2), diff(dz,q3), diff(dz,q4)];

Jv = [vx; vy; vz];

R = T05(1:3,1:3);

syms dotq1 dotq2 dotq3 dotq4 real

dotR = simplify(   diff(R,q1) * dotq1 + diff(R,q2) * dotq2...
                 + diff(R,q3) * dotq3 + diff(R,q4) * dotq4);

Omega = dotR * inv(R); 
Omega = simplify(Omega);

wx = Omega(3,2);
wy = Omega(1,3);
wz = Omega(2,1);

Jw = [0 0 0 -sin(q2);
      0 1 0  0;
      0 0 0 -cos(q2)];
   
JdotR = [Jv; Jw]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 3 
% 6 x 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 4
% Cuando se expresa la orientación en ángulos de Euler ZYX o en RPY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Bonos
%  5.15_04.00.4001
% 1. Calibrar con 3 o más puntos
% 2. Calibrar con 3 o más puntos y eje z
% 4. Calibrar con 3 o más puntos y ejes z, x
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


