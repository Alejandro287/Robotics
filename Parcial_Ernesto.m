%% Examen 4: Jacobiano
% Alejandro Cano Rico
%%
clear all 
clc
syms q1 q2 q3 q4 q5  L1 L2 L3 L4 L5 L6 L7

LL(1) = Link('prismatic' ,'alpha',0,    'a',0, 'theta',0  ,'offset',0, 'qlim', [0 100], 'modified');
LL(2) = Link('revolute'  ,'alpha',pi/2, 'a',0, 'd',-L2       ,'offset',0, 'modified');
LL(3) = Link('prismatic' ,'alpha',pi,   'a',L3,'theta',0     ,'offset',0, 'qlim', [0 100], 'modified');
LL(4) = Link('revolute'  ,'alpha',-pi/2,'a',0, 'd',L5         ,'offset',0, 'modified');
LL(5) = Link('revolute'  ,'alpha',0,    'a',L7, 'd',L6        ,'offset',0, 'modified');

PRPR = SerialLink(LL,'name','PRPR');

%%
A01 = simplify(LL(1).A(q1));
A12 = simplify(LL(2).A(q2));
  A12(3,3)=0;
  A12(3,4)=0;
    A12(2,2)=0;
% % %  A12(3,2)=0;
    A12(2,1)=0;
% % %  A12(1,1)=0;
A23=simplify(LL(3).A(q3));
% %  A23(1,1)=0;
% %  A23(2,1)=0;
% %  A23(2,2)=0;
   A23(3,2)=0;
  A23(2,3)=0;
  A23(2,4)=0;
A34=simplify(LL(4).A(q4));
  A34(2,1)=0;
  A34(2,2)=0;
 A34(3,3)=0;
 A34(3,4)=0;
A45=simplify(LL(5).A(q5));

TCP=simplify(A01*A12*A23*A34*A45);

%% Jacobiano Convencional

dx = TCP(1,4);  % componente x
dy = TCP(2,4);  % componente y
dz = TCP(3,4);  % componente z

vx = [diff(dx,q1), diff(dx,q2), diff(dx,q3), diff(dx,q4), diff(dx,q5)];
vy = [diff(dy,q1), diff(dy,q2), diff(dy,q3), diff(dy,q4), diff(dy,q5)];
vz = [diff(dz,q1), diff(dz,q2), diff(dz,q3), diff(dz,q4), diff(dz,q5)];

Jv = [vx; vy; vz];

R = TCP(1:3,1:3);

syms dq1 dq2 dq3 dq4 dq5 real

dotR = simplify(diff(R,q1)*dq1 + diff(R,q2)*dq2 + diff(R,q3)*dq3 + diff(R,q4)*dq4++ diff(R,q5)*dq5);
Omega = simplify(dotR*inv(R)); % Transpuesta de la matriz de rotaci?n


wx = Omega(3,2);
wy = Omega(1,3);
wz = Omega(2,1);

%%%%%%wx            wy  wz  
Jw1 = [0       0        0]'; % dotq1 Columnas para dotq1
Jw2 = [0      -1        0]'; % dotq2 Columnas para dotq2
Jw3 = [0       0        0]'; % dotq3 Columnas para dotq3 (No influyen en la velocidad angular)
Jw4 = [sin(q2) 0 -cos(q2)]'; % dotq4
Jw5 = [sin(q2) 0 -cos(q2)]'; % dotq5

Jw = [Jw1 Jw2 Jw3 Jw4 Jw5];

% Por ultimo organizando tanto el Jv y Jw se tiene
J0 = [Jv; Jw]
Xdot=J0*[dq1 dq2 dq3 dq4 dq5]';
%% Valores
L1=0;L2=1;L3=1;L4=0;L5=1;L6=0.5;L7=0.5;
q1=2;q2=pi/12;q3=0.5;q4=pi/4;q5=0;




LL(1) = Link('prismatic' ,'alpha',0,    'a',0, 'theta',0  ,'offset',0, 'qlim', [0 100], 'modified');
LL(2) = Link('revolute'  ,'alpha',pi/2, 'a',0, 'd',-L2       ,'offset',0, 'modified');
LL(3) = Link('prismatic' ,'alpha',pi,   'a',L3,'theta',0     ,'offset',0, 'qlim', [0 100], 'modified');
LL(4) = Link('revolute'  ,'alpha',-pi/2,'a',0, 'd',L5         ,'offset',0, 'modified');
LL(5) = Link('revolute'  ,'alpha',0,    'a',L7, 'd',L6        ,'offset',0, 'modified');

PRPR = SerialLink(LL,'name','PRPR');

q = [2 pi/12 0.5 pi/4 0];
q = [0 0 0 0 0];

PRPR.plot(q,'workspace',[-3 3 -3 3 -3 3],'scale',0.5);
axis([-3 3 -3 3 -3 3]);
PRPR.teach();

JACOB=PRPR.jacob0([q1 q2 q3 q4 q5]);

Q = [q1 q2 q3 q4 q5];

eval(J0)
disp(JACOB)

q1=0;q2=0;q3=0;q4=0;q5=0;
dq1=1;dq2=0;dq3=1;dq4=0;dq5=0;

eval(Xdot)

%% Punto 1 

% Un jacobiano no cuadrado no es invertible

%% Punto 3

% es de 6X7
%% Punto 4
% seran iguales si el analitico tiene su orientacion en angulos de euler
% xyz

%% Bono 

%la versión de robotware que se encuentra instalada en el flex pendant es la 5.1504

% 1 eje y 3 puntos
% 3 puntos 
%