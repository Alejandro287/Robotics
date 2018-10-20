%% Anexo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   UNIVERSIDAD NACIONAL DE COLOMBIA                    %
%                   ROBÓTICA - Taller No. 4                             %
%                   ALEJANDRO CANO RICO - 25481055                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% PUNTO 1 

% Propiedades jacob0- jacobiano respecto a la base
% j0 = R.jacob0(q, options) 

% 'rpy' Calcula el jacobiano analítico con velocidad de rotación 
%     en términos de ángulos de  roll-pitch-yaw
% 'eul' Calcula el Jacobiano analítico  con tasas de rotación en 
%     términos de ángulos de Euler
% 'trans' Devuelve la submatriz translacional del Jacobiano
% 'rot' Devuelve la Submatriz rotacional de Jacobiano

% Propiedades jacobn - jacobiano respecto a la herramienta
% jn = R.jacobn(q, options) 

% 'trans' Devuelve la submatriz translacional jacobiana
% 'podredumbre' Devuelve submatriz rotacional jacobiana


%% PUNTO 2

%   El elipsoide de manipulabilidad es la capacidad de cambio en posición
%   y orientación del efector final de un sistema robótico en una confi-
%   guración dada.
%
%   La manipulabilidad puede verse como un elipsoide en el espacio
%   Euclidiano n-dimensional cuya geometría puede definirse al resolver
%   la siguiente inecuación.')
%
%   sqrt((q1^2)+(q2^2)+ . . . +(qn^2)) <= 1
%   Ésta, representa el conjunto de todas las velocidades que son reali-
%   zables por una articulación tal que la norma Euclídea de   es infe-
%   rior a la unidad. De dicha inecuación se tiene que el efector final
%   dispone de mayor capacidad de movimiento en la dirección del eje mayor
%   del elipsoide. Por otra parte, en la dirección del eje menor la
%   capacidad de desarrollar velocidad será menor. Este elipsoide simbo-
%   liza la capacidad de manipulación y se le conoce como Elipsoide de
%   Manipulabilidad.

%% PUNTO 3

% El numero de condicion permite obtener un valor cuantitativo asociado con el desempeño 
% cinetostatico del manipulador en determinadas poses de su espacio de trabajo. El valor 
% minimo que puede tomar el numero de condicion es la unidad, en ese caso se dice que esa 
% pose es isotropica y evidentemente es la mejor condicion esperada. Por otro lado, el 
% numero de condicion puede tender a infinito y en ese caso se dice que la pose del 
% manipuladores singular.

%% PUNTO 4

% La formulacion para el modelo de cinematica directa al plantearse de
% manera inversa, al tener muchos senos y cosenos es bastante complejo y
% habitualmente analiticamente irrealizable.

% La Jacobiana puede considerarse como el núcleo de una aproximación lineal 
% del modelo cinematico del manipulador, es decir,por  su  propia  definicion 
% la  Jacobiana  no  dice  aproximadamente  cuanto  cambia E en  el  espacio
% cartesiano  en  el  entorno  de  un  punto  articular q0  ante  variaciones 
% articulares.  De  hecho,  cada  columna  de  la  Jacobiana  nos  informa 
% del  cambio  que  sufre  el  extremo  por  la  variación  de  la  
% correspondiente  variable  articular.  Esta  estimacion (como  en  cualquier  
% aproximacion  tangencial)  sera peor en la medida en que nos alejamos mas 
% del punto en el que se ha calculado la Jacobiana, que  normalmente  expresamos
% en  base  al  incremento  infinitesimal  respecto  del  tiempo,  pero  
% que  nos  muestra  la  aproximación  incremental.  Es  decir,  respecto  de  un  punto  
% E0, q0 se puede escribir como:

% delta(E)=J(q0)*delt(q)
m1=imread('jacobiano.jpg');
figure (1)
imshow(m1);

%% PUNTO 5

clear all
clc

syms q1 q2 q3 L1 L2

clear all;
clc;
syms q1 q2 q3 L1 L2 real
% L1 = 2;
% L2 = 2;


L(1) = Link('revolute' ,'alpha',0,    'a',0, 'd',0     ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',L1, 'd',0     ,'offset',0, 'modified');
RR = SerialLink(L,'name','2R')
RR.tool=transl(L2,0,0);

A01 = simplify(L(1).A(q1));
A12 = simplify(L(2).A(q2));
A23 = (transl(L2,0,0));

%Cinematica Directa
A03 = simplify(A01*A12*A23);


% Calculo de los 0Zi
z01 = A01(1:3,3);           % Articulación 1
A02 = simplify(A01*A12); 
z02 = A02(1:3,3);           % Articulación 2
A03 = simplify(A02*A23);
z03 = A03(1:3,3);           % Efector Final


% Calculo del vector ^iP_n
p23 = A03(1:3,4) - A02(1:3,4);
p13 = A03(1:3,4) - A01(1:3,4);


% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional 

disp('Jacobiano Geometrico o Convencional')
Jgeo = simplify([J1, J2])       % Jacobiano Geometrico




disp('Jacobiano geometrico por propagacion de velocidades') 
T01=A01;
T12=A12;
T23=A23;
T03=A03;

syms q1p q2p real


v00 = [0 0 0]';
w00 = [0 0 0]';
w11 = T01(1:3,1:3)'*w00+q1p*T01(1:3,3);
v11 = T01(1:3,1:3)'*(v00 + simplify(cross(w00,T01(1:3,4))));
w22 = T12(1:3,1:3)'*w11+q2p*T12(1:3,3);
v22 = simplify(T12(1:3,1:3)'*(v11 + simplify(cross(w11,T12(1:3,4)))));
w33 = T23(1:3,1:3)'*w22+0*T23(1:3,3);
v33 = simplify(T23(1:3,1:3)'*(v22 + cross(w22,T23(1:3,4))));
v03 = simplify(T03(1:3,1:3)*v33)
w03 = w33
% Se obtiene el jacobiano al poner el vector [v03;w03] en terminos  
J1g= [v03;w03];
J1g=subs(J1g,q1p,1);
J1g=subs(J1g,q2p,0);

J2g= [v03;w03]
J2g=subs(J2g,q1p,0);
J2g=subs(J2g,q2p,1);


disp('Jacobiano Analitico')

Jgeo2=simplify([J1g, J2g])
% Jacobiao analitico

pos_x = T03(1,4);
pos_y = T03(2,4);

% Casillas del Jacobiano

J11 = diff(pos_x,q1);
J12 = diff(pos_x,q2);

J21 = diff(pos_y,q1);
J22 = diff(pos_y,q2);


Jsym = [J11 J12 ;
        J21 J22];
    

L1 = 2;
L2 = 2;
q1 = pi/12;
q2 = pi/6;

disp('Jacobiano Geometrico Evaluado')
Jgeo_eval = eval(Jgeo)          % Jacobiano Geometrico Evaluado

q1_punto = pi/4;
q2_punto = pi/2;
q_punto = [q1_punto; q2_punto];

disp('b. Vector de Velocidades')
VW = Jgeo_eval*q_punto



%%

L1 = 2;
L2 = 2;
l=L1;
L(1) = Link('revolute' ,'alpha',0,    'a',0, 'd',0     ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',L1, 'd',0     ,'offset',0, 'modified');
dosr = SerialLink(L,'name','2R')
dosr.tool=transl(L2,0,0);
vector=[2.8*ones(1,10);linspace(1,2.8,10)];
q_elip = ones(2,10);
for i=1:length(vector)
    x = vector(1,i);
    y = vector(2,i);
    D = (x^2 + y^2 - l^2 -l^2)/(2 *l* l);
    if abs(D)>1
        disp('D>1');
        Q1=NaN;
        Q2=NaN;
    else
        q2_1=atan2(sqrt(1-D^2),D);
        q2_2=atan2(-sqrt(1-D^2),D);
        q1_1 = atan2(y,x) - atan2(l * sin(q2_1),l + l * cos(q2_1));
        q1_2 = atan2(y,x) - atan2(l * sin(q2_2),l + l * cos(q2_2));
        Q1=[q1_1,q2_1];
        Q2=[q1_2,q2_2];
        q_elip(1,i)=q1_1;
        q_elip(2,i)=q2_1;
    end
end

figure
grid on
hold on
trplot(eye(4),'rgb')
axis([-6 6 -6 6])
for i=1:length(vector)
    j=dosr.jacob0(q_elip(:,i));
    j=[j(1,:);j(2,:)];
    c=dosr.fkine(q_elip(:,i));
    dosr.plot(q_elip(:,i)')
    plot_ellipse(j*j',c(1:2,4))
    pause(1)
end

%% PUNTO 6

clear all
clc
warning('off','all')
syms q1 q2 q3 q4 l1 l2 l3 real
L(1) = Link('revolute','alpha',0,'a',0,'d',l1,'offset',0,'modified', 'sym');
L(2) = Link('revolute','alpha',-pi/2,'a',0,'d',0,'offset',0,'modified', 'sym');
L(3) = Link('revolute','alpha',0,'a',l2,'d',0,'offset',-pi/2,'modified', 'sym');
L(4) = Link('revolute','alpha',0,'a',l3,'d',0,'offset',0,'modified', 'sym');

tresR=SerialLink(L, 'name','tresR')


M0A1=L(1).A(q1);
M1A2=L(2).A(q2);
M2A3=L(3).A(q3);
M3A4=L(4).A(q4);

z01 = M0A1(1:3,3); % art 1
M0A2 = M0A1*M1A2;
z02 = M0A2(1:3,3); % Art 2
M0A3=M0A2*M2A3;
z03=M0A3(1:3,3);
M0A4=M0A3*M3A4;
z04=M0A4(1:3,3);

p34 = M0A4(1:3,4) - M0A3(1:3,4);
p24 = M0A4(1:3,4) - M0A2(1:3,4);
p14 = M0A4(1:3,4) - M0A1(1:3,4);
p44 = M0A4(1:3,4) - M0A4(1:3,4);

J1 =simplify([(skew(z01) * p14); z01]);  % Rotacional
J2 = simplify([(skew(z02) * p24); z02]); % Rotacional 
J3 =simplify([(skew(z03) * p34); z03]); % Rotacional
J4 = simplify([(skew(z04) * p44); z04]); % Rotacional

%jacobiano geométrico
   j0n=simplify([J1,J2,J3,J4])
   
q1=pi/4;
q2=pi/4;
q3=pi/2;
q4=0;
l2=1;
l3=1;
l1=1;

j0n =eval(j0n)




l = [1 1 1];
q = [pi/4 pi/4 pi/2 0];

L(1) = Link('revolute','alpha',0,'a',0,'d',l(1),'offset',0,'modified');
L(2) = Link('revolute','alpha',pi/2,'a',0,'d',0,'offset',0,'modified');
L(3) = Link('revolute','alpha',0,'a',l(2),'d',0,'offset',0,'modified');
L(4) = Link('revolute','alpha',0,'a',l(3),'d',0,'offset',0,'modified');
tres=SerialLink(L, 'name','tresR');
l1 = l(1);
l2 = l(2);
l3 = l(3);
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

j0 = (tres.jacob0([q1 q2 q3 q4]))


 

%% Punto 7
clear all
clc

syms q1 q2 q3 q4 q5 q1p q2p q3p q4p q5p l1 real
% l1 = 1;
L(1) = Link('prismatic','alpha',0,'a',0,'theta',0,'offset',0,'qlim',[0 100],'modified');
L(2) = Link('prismatic','alpha',-pi/2,'a',0,'theta',-pi/2,'offset',0,'qlim',[0 100],'modified');
L(3) = Link('prismatic','alpha',-pi/2,'a',0,'theta',0,'offset',0,'qlim',[0 100],'modified');
L(4) = Link('revolute','alpha',-pi/2,'a',0,'d',0,'offset',0,'modified');
L(5) = Link('revolute','alpha',0,'a',l1,'d',0,'offset',0,'modified'); 
PPPR = SerialLink(L, 'name','PPPR');


% CinemÃ¡tica directa
T01 = simplify(L(1).A(q1));
T12 = simplify(L(2).A(q2));
T12(1,1) = 0;
T12(2,1) = 0;
T12(2,2) = 0;
T12(3,2) = 0;
T12(3,3) = 0;
T12(3,4) = 0;
T23 = simplify(L(3).A(q3));
T23(2,2) = 0;
T23(3,3) = 0;
T23(3,4) = 0;
T34 = simplify(L(4).A(q4));
T34(2,1) = 0;
T34(2,2) = 0;
T34(3,3) = 0;
T45 = simplify(L(5).A(q5));

T02 = simplify(T01*T12);
T03 = simplify(T01*T12*T23);
T04 = simplify(T01*T12*T23*T34);
T05 = simplify(T01*T12*T23*T34*T45);

% Vector de posicion
x = T05(1,4);
y = T05(2,4);
z = T05(3,4);
% Orientacion en Euler ZYZ

% Orientacion en Euler ZYZ
phi = pi/2;
theta = pi/2;
psi = q4 + q5;

% Coordenadas generalizadas
X = [x; y; z; phi; theta; psi];
% Jacobiano Analitico
% Se deriva por columnas 6 x 1.

J1 = [diff(x, q1) diff(y, q1) diff(z, q1) diff(phi, q1) diff(theta, q1) diff(psi,q1)]';
J2 = [diff(x, q2) diff(y, q2) diff(z, q2) diff(phi, q2) diff(theta, q2) diff(psi,q2)]';
J3 = [diff(x, q3) diff(y, q3) diff(z, q3) diff(phi, q3) diff(theta, q3) diff(psi,q3)]';
J4 = [diff(x, q4) diff(y, q4) diff(z, q4) diff(phi, q4) diff(theta, q4) diff(psi,q4)]';
J5 = [diff(x, q5) diff(y, q5) diff(z, q5) diff(phi, q5) diff(theta, q5) diff(psi,q5)]';
% Jacobiano analitico
Janalitico = [J1 J2 J3 J4 J5]

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
J1 =simplify([z01              ;zeros(3,1)]); % Rotacional
J2 =simplify([z02              ;zeros(3,1)]); % Prismatica
J3 =simplify([z03              ;zeros(3,1)]); % Prismatica
J4 =simplify([(skew(z04) * p45);      z04]); % Rotacional 
J5 =simplify([(skew(z05) * p55);      z05]); % Rotacional 

Jgeo = simplify([J1, J2, J3, J4, J5])

% * Dada la configuraciÃ³n [q1 q2 q3 q4 ] = [1 1 1 pi/2]
% y el parametro l1 = 1m, determine J0 .

l1 = 1
q = [1 1 1 pi/2 0];

L(1) = Link('prismatic','alpha',0,'a',0,'theta',0,'offset',0,'qlim',[0 100],'modified');
L(2) = Link('prismatic','alpha',-pi/2,'a',0,'theta',-pi/2,'offset',0,'qlim',[0 100],'modified');
L(3) = Link('prismatic','alpha',-pi/2,'a',0,'theta',0,'offset',0,'qlim',[0 100],'modified');
L(4) = Link('revolute','alpha',-pi/2,'a',0,'d',0,'offset',0,'modified');
L(5) = Link('revolute','alpha',0,'a',l1,'d',0,'offset',0,'modified'); 
P3R = SerialLink(L, 'name','3PR');

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);

Jgeo =eval(Jgeo)

j0 = (P3R.jacob0([q1 q2 q3 q4 q5]))

% * Dado el vector q punto= [1,5 1,2 1,8 pi/4 ]' encuentre áºŠ usando la configuraciÃ³n del item anterior.
% Â¿Que significado tiene el vector de velocidades generalizados? (Explique su respuesta).

q_dot = [1.5 1.2 1.8 pi/4 0]';
X = Jgeo*q_dot

% se puede observar en la respuesta que tiene velocicdad en los 3 ejes
% cartesianos, al tener 1 articualicon prismatica en cada uno, y tendra una
% velocidad angular en el eje y, que es coo se encuentra dispuesta la
% cuarta articulacion, la cual es rotacional.

