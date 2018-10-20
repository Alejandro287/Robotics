% Toolbox
% Robot SCARA
L(1) = Link('revolute'  ,'alpha',0,    'a',0,  'd',1     ,'offset',0, 'modified');
L(2) = Link('revolute'  ,'alpha',0,    'a',1, 'd',0     ,'offset',0, 'modified');
L(3) = Link('prismatic' ,'alpha',0,    'a',1, 'theta',0 ,'offset',0, 'qlim', [0 1], 'modified');
L(4) = Link('revolute'  ,'alpha',pi,   'a',0, 'd',1     ,'offset',0, 'modified');
SCARA = SerialLink(L,'name','scara');

SCARA.plot([0 0 0 0],'workspace',[-3 3 -3 3 -3 3],'scale',0.6)
SCARA.teach();
%%
% SCARA simbolico
syms q1 q2 q3 q4 L1 L2 L3 L4 as real
L(1) = Link('revolute'  ,'alpha',0,    'a',0,  'd',L1    ,'offset',0, 'modified');
L(2) = Link('revolute'  ,'alpha',0,    'a',L2, 'd',0     ,'offset',0, 'modified');
L(3) = Link('prismatic' ,'alpha',0,    'a',L3, 'theta',0 ,'offset',0, 'modified');
L(4) = Link('revolute'  ,'alpha',pi,   'a',0,  'd',L4    ,'offset',0, 'modified');
SCARAsym = SerialLink(L,'name','scara');

%%
% Cinemática directa
T01 = simplify(L(1).A(q1));
T12 = simplify(L(2).A(q2));
T23 = simplify(L(3).A(q3));
T34 = simplify(L(4).A(q4));
T34(3,1) = 0;
T34(3,2) = 0;
T34(2,3) = 0;
T34(2,4) = 0;

T04 = simplify(T01*T12*T23*T34);
%% Cinematica directa
CD  = simplify(T04)

% Vector de posicion
x = CD(1,4);
y = CD(2,4);
z = CD(3,4);
% Orientacion en Euler ZYZ
phi = (q1 + q2 - q4);
theta = pi;
psi = pi;

% Coordenadas generalizadas
X = [x; y; z; phi; theta; psi];

%% Jacobiano Analitico
% Se deriva por columnas 6 x 1.
J1 = [diff(x, q1) diff(y, q1) diff(z, q1) diff(phi, q1) diff(theta, q1) diff(psi,q1)]';
J2 = [diff(x, q2) diff(y, q2) diff(z, q2) diff(phi, q2) diff(theta, q2) diff(psi,q2)]';
J3 = [diff(x, q3) diff(y, q3) diff(z, q3) diff(phi, q3) diff(theta, q3) diff(psi,q3)]';
J4 = [diff(x, q4) diff(y, q4) diff(z, q4) diff(phi, q4) diff(theta, q4) diff(psi,q4)]';

% Jacobiano analitico
Janalitico = [J1 J2 J3 J4]

%% Jacobiana Geometrico 
% (Derivando matriz de Rotacion)
T = CD;

% Se determina el Jacobiano de posicion Jv
dx = T(1,4);  % componente x
dy = T(2,4);  % componente y
dz = T(3,4);  % componente z

% Derivando con respecto al tiempo
% Nota: Se aplica la regla de la cadena
vx = [diff(dx,q1), diff(dx,q2), diff(dx,q3), diff(dx,q4)]; 
vy = [diff(dy,q1), diff(dy,q2), diff(dy,q3), diff(dy,q4)];
vz = [diff(dz,q1), diff(dz,q2), diff(dz,q3), diff(dz,q4)];

% Se tiene el Jacobiano de posicion Jv 
Jv = [vx; vy; vz];

% Se determina el Jacobiano de Orientacion Jw
% Tomando la Matriz de rotacion de T
R = T(1:3,1:3)

% Derivando R se tiene
% 1. Las derivadas de q con respecto al tiempo simbolicas
syms dotq1 dotq2 dotq3 dotq4 real

% 2. La derivada de R resulta
dotR = simplify(   diff(R,q1) * dotq1 + diff(R,q2) * dotq2...
                 + diff(R,q3) * dotq3 + diff(R,q4) * dotq4)

% Se determina la matriz skew
Omega = dotR * inv(R) % Transpuesta de la matriz de rotaci?n
Omega = simplify(Omega)

% Logrando el vector w = [wx wy wz]'
wx = Omega(3,2)
wy = Omega(1,3)
wz = Omega(2,1)

% Organizando la Matriz jacobiana de orientacion Jw y sabiendo que las
% velocidades angular resultan de una combinacion lineal de ellas se tiene:
Jw1 = [0 0 1]'; % Columnas para dotq1
Jw2 = [0 0 1]'; % Columnas para dotq2
Jw3 = [0 0 0]'; % Columnas para dotq3 (No influyen en la velocidad angular)
Jw4 = [0 0 -1]'; % Columnas para dotq4

Jw = [Jw1 Jw2 Jw3 Jw4];

% Por ultimo organizando tanto el Jv y Jw se tiene
Jgeo_derR = [Jv; Jw]