clear all
clc
% Robot
syms q1 q2 q3 q4 q5 l1 l2 l3 l4 l5 l6 l7 real

L(1) = Link('prismatic' ,'alpha',0,    'a',0, 'theta',0  ,'offset',0, 'qlim', [0 100], 'modified');
L(2) = Link('revolute'  ,'alpha',pi/2, 'a',0, 'd',-l2       ,'offset',0, 'modified');
L(3) = Link('prismatic' ,'alpha',pi,   'a',l3,'theta',0     ,'offset',0, 'qlim', [0 100], 'modified');
L(4) = Link('revolute'  ,'alpha',-pi/2,'a',0, 'd',l5         ,'offset',0, 'modified');
L(5) = Link('revolute'  ,'alpha',0,    'a',l7, 'd',l6        ,'offset',0, 'modified');
robot = SerialLink(L,'name','robot');
robot.tool();

% Geometrico

% Cinemática directa
T01 = simplify(L(1).A(q1));
T01(1,1) = 0;
T01(2,2) = 0;
T12 = simplify(L(2).A(q2));
T12(2,1) = 0;
T12(2,2) = 0;
T12(3,3) = 0;
T12(3,4) = 0;
T23 = simplify(L(3).A(q3));
T23(2,2) = 0;
T23(3,3) = 0;
T23(3,4) = 0;
T34 = simplify(L(4).A(q4));
T45 = simplify(L(5).A(q5));

T02 = simplify(T01*T12);
T03 = simplify(T01*T12*T23);
T04 = simplify(T01*T12*T23*T34);
T05 = simplify(T01*T12*T23*T34*T45);

% Vector de posicion
x = T05(1,4);
y = T05(2,4);
z = T05(3,4);


% Jacobiano geometrico
% Calculo de los 0Zi 
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
z03 = T03(1:3,3); % Art 3
z04 = T04(1:3,3); % Art 4
z05 = T05(1:3,3); % Art 5
% Calculo del vector ^iP_n
p55 = T05(1:3,4) - T05(1:3,4);
p45 = T05(1:3,4) - T04(1:3,4);
p35 = T05(1:3,4) - T03(1:3,4);
p25 = T05(1:3,4) - T02(1:3,4);
p15 = T05(1:3,4) - T01(1:3,4);

% Calculo de las columnas del jabobiano
J1 =simplify([z01              ;zeros(3,1)]); % Prismatica
J2 =simplify([(skew(z02) * p25);      z02]); % Rotacional
J3 =simplify([z03              ;zeros(3,1)]); % Prismatica
J4 =simplify([(skew(z04) * p45);      z04]); % Rotacional 
J5 =simplify([(skew(z05) * p55);      z05]); % Rotacional
Jgeo = simplify([J1, J2, J3, J4, J5])

l = [2 1 1 1];
q = [1 pi/4 0.5 pi/2 0];

l1 = l(1);
l2 = l(2);
l3 = l(3);
l4 = l(4);
l5 = l(4);
l6 = l(4);
clear all
clc
% Robot


L(1) = Link('prismatic' ,'alpha',0,    'a',0, 'theta',0  ,'offset',0, 'qlim', [0 100], 'modified');
L(2) = Link('revolute'  ,'alpha',pi/2, 'a',0, 'd',-l2       ,'offset',0, 'modified');
L(3) = Link('prismatic' ,'alpha',pi,   'a',l3,'theta',0     ,'offset',0, 'qlim', [0 100], 'modified');
L(4) = Link('revolute'  ,'alpha',-pi/2,'a',0, 'd',l5         ,'offset',0, 'modified');
L(5) = Link('revolute'  ,'alpha',0,    'a',l7, 'd',l6        ,'offset',0, 'modified');
robot = SerialLink(L,'name','robot');
robot.tool();


robot.plot(qt,'workspace',escala*[-1000 1000 -1000 1000 -1000 1000],'tilesize',100,'scale',0.35,'noa');
axis(escala*[-1000 1000 -1000 1000 -100 1000]);



 