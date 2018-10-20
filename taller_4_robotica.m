%% Taller 4 - Jacobiano
% Julio Cesar Ceron Espejo
% Cod: 25481067
%% Punto 1
% Explique las propiedades jacob0 y jacobn del objeto SerialLink del toolbox de Peter Corke,
% describa las diferencias entre ellas.
%
% -jacob0 (jacobiano respecto a la base): Al usar esta propiedad, se obtiene la matriz Jacobiana de tamaño 6xN para el
% robot o manipulador en una pose dada por el vector de valores articulares q de tamaño
% 1xN. La matriz Jacobiana del manipulador mapea en conjunto la velocidad espacial
% del efector final con la velocidad V de cada articulación, expresados en el sistema de
% coordenadas mundo o base.
% Para esta propiedad, el Jacobiano es calculado en el sistema de coordenadas base y es
% transformado al sistema coordenado del efector final. Ademas, el Jacobiano predetermi-
% nado devuelto se refiere a menudo al Jacobiano geométrico, como opuesto al Jacobiano
% analı́tico.
%
% -jacobn(jacobiano respecto al efector final): Al usar esta propiedad, se obtiene la matriz Jacobiana de tamaño 6xN para el
% robot o manipulador en una pose dada por el vector de valores articulares q de tamaño
% 1xN. La matriz Jacobiana del manipulador mapea en conjunto la velocidad espacial
% del efector final con la velocidad V de cada articulación, expresados en el sistema de
% coordenadas del efector final.
% Para este caso, usando esta propiedad, el Jacobiano calculado a menudo hace referencia
% al Jacobiano Geométrico.
%% Punto 2
% Averigüe en qué consiste el indice de manipulabilidad de Yoshikawa, ¿existen otros valores aso-
% ciados a la manipulabilidad de un robot?.

% Los indices de desempeño cinetostático permiten evaluar la capacidad del manipulador para
% transformar las velocidades o fuerzas en los actuadores, en velocidades o fuerzas en el efector
% final. La mayoria de estos indices están definidos en funcion de la matriz Jacobiana del
% manipulador, la cual establece las relaciones de velocidad y fuerza entre las articulaciones y
% el efector final.

%́ * Indice de la manipulabilidad:
% El ındice de la manipulabilidad fue propuesto en (Yoshikawa,1985). El proposito de
% este indice es medir la capacidad de un robot, en cierta configuracion, para generar
% velocidades en el efector final. Este indice de desempeño es proporcional al volumen
% del elipsoide de velocidad. Para el caso general (incluyendo robots redundantes) la
% manipulabilidad esta definida de la siguiente manera:
%
% w = sqrt(det(JJ'))
%
% Para robots no redundantes se tiene que w = |det(J)|. La manipulabilidad es equivalente
% al producto de los valores singulares, w = σ1σ2...σm. A mayores valores de la manipu-
% labilidad el robot tiene una mayor capacidad de realizar movimientos en el efector final.
% En el caso de que la manipulabilidad sea igual a cero entonces el robot se encuentra
% en una configuración singular. En esta configuracion el robot pierde la capacidad de
% realizar movimientos en ciertas direcciones.
% * Elipsoide de Velocidades:
% 
% El elipsoide de velocidad fue propuesto inicialmente en (Yoshikawa, 1985b) para la
% definicion del indice de la manipulabilidad, sin embargo tambien resulta util para la
% definicion de otros indices de desempeño cinematico. Para comenzar considerese el
% modelo de velocidad de un robot manipulador:
%
% Ẋ = J(q punto)
%
% El elipsoide de velocidad esta dado por el conjunto de todas la velocidades del efector
% final que son realizables para las velocidades articulares que satisfacen que ||q punto|| ≤ 1.
% Del elipsoide de velocidad se puede observar que el robot tiene la capacidad de moverse
% a mayor velocidad cuanto mayor sea el volumen del elipsoide. Para las velocidades en
% el espacio articular que cumplan con ||q punto|| ≤ 1, las mayores velocidades que el efector
% final puede alcanzar, se obtienen sobre el semieje mayor del elipsoide. Por el contrario,
% el efector final del robot se movera a baja velocidad en direccion del semieje menor del
% elipsoide. Por otro lado, si la forma del elipsoide se aproxima a la esfera, entonces el
% efector final se puede mover uniformemente en todas direcciones.
%% Punto 3
% ¿Que significado tiene que el numero de condicion de una matriz sea mayor a 1?

% El numero de condicion permite obtener un valor cuantitativo asociado con el desempeño 
% cinetostatico del manipulador en determinadas poses de su espacio de trabajo. El valor 
% minimo que puede tomar el numero de condicion es la unidad, en ese caso se dice que esa 
% pose es isotropica y evidentemente es la mejor condicion esperada. Por otro lado, el 
% numero de condicion puede tender a infinito y en ese caso se dice que la pose del 
% manipuladores singular.
%% Punto 4
% Consulte de que manera se puede usar el Jacobiano para calcular la cinemática inversa de un
% robot.
%
% Consideremos que E = xi(q) es la solución cinematica del robot, por lo que
% Ek  es una representación de la  localizacion  espacial  del  extremo.  Para  este caso
% no  se  considerará  el  comportamiento particular de la orientacion, por lo que sea va
% a considerar que son posiciones para facilitar la exposicion. 
% El problema cinemático inverso busca obtener xi^-1 de forma que:
% q = xi⁻1(E) 
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
m1=imread('jacobiano.png');
figure (1)
imshow(m1);
%% Punto 5
% Dado un manipulador 2R con longitudes de eslabon l 1 = l 2 = 2:
% * Determine los Jacobiano analitico, geometrico y geometrico por propagacion de velocidades.
clear all;
clc;
syms q1 q2 q3 q1p q2p real
% L1 = 2;
% L2 = 2;

L(1) = Link('revolute' ,'alpha',0,    'a',0, 'd',0     ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',L1, 'd',0     ,'offset',0, 'modified');
RR = SerialLink(L,'name','2R')
RR.tool=transl(L2,0,0);

% q1 = 0;
% q2 = 0;
% q =  [q1 q2];
% 
% figure (1)
% title('Manipulador en posicio inicial')
% trplot(eye(4),'length',1,'rgb');
% hold on
% RR.plot(q,'workspace',[-10 10 -10 10 -10 10],'scale',0.5,'jaxes');
% axis([-5 5 -5 5 -5 5])
% RR.teach();

% Jacobiano geometrico

% Cinemática directa
T01 = simplify(L(1).A(q1));
T12 = simplify(L(2).A(q2));
T23 = (transl(L2,0,0));
T02 = simplify(RR.A([1 2],[q1 q2]));
T03 = simplify(RR.A([1 2],[q1 q2])*T23);
% Calculo de los 0Zi
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
% Calculo del vector ^iP_n
p23 = T03(1:3,4) - T02(1:3,4);
p13 = T03(1:3,4) - T01(1:3,4);
% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional
Jgeo = simplify([J1, J2])

% Jacobiano geometrico por propagacion de velocidades
% se tomo q1 punto y q2 punto, simbolicamente como q1p y q2p
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

J2g= [v03;w03];
J2g=subs(J2g,q1p,0);
J2g=subs(J2g,q2p,1);

Jgeo2=simplify([J1g, J2g])
% Jacobiao analitico

pos_x = T03(1,4);
pos_y = T03(2,4);

% Casillas del Jacobiano
% se deriva con respectivo a q3 como una articulacion falsa
J11 = diff(pos_x,q1);
J12 = diff(pos_x,q2);

J21 = diff(pos_y,q1);
J22 = diff(pos_y,q2);


Jsym = [J11 J12 ;
        J21 J22]
    
%
% * El vector de velocidades en el efector final Ẋ sabiendo que: q punto=
% [pi/4, pi/2] y q = [pi/12, pi/3]

q = ([pi/12 pi/3]);
dot_x = [pi/4 pi/2]';

q1 = q(1);
q2 = q(2);
Jsym_0 = eval(Jsym);
Jgeo_0 = eval(Jgeo);
Jgeo2_0 = eval(Jgeo2);
Jinv = simplify(inv(Jsym));
Jinv_0 = eval(Jinv);

% Velocidad del efector final en x y y
velTCP=Jinv_0*dot_x

% * Genere una trayectoria lineal en el espacio de p 1 = [3 2] hasta p 2 = [3 3] con al menos 10
% puntos intermedios, para cada configuracion del robot grafique la elipse
% de velocidades, ¿que significado tiene una elipse más alargada, mas simétrica?


%
L1 = 2;
L2 = 2;
l=L1;
L(1) = Link('revolute' ,'alpha',0,    'a',0, 'd',0     ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',L1, 'd',0     ,'offset',0, 'modified');
dosr = SerialLink(L,'name','2R')
dosr.tool=transl(L2,0,0);
vector=[2.8*ones(1,10);linspace(1,2.8,10)]
q_elip = ones(2,10);
for i=1:length(vector)
    x = vector(1,i)
    y = vector(2,i)
    D = (x^2 + y^2 - l^2 -l^2)/(2 *l* l);
    if abs(D)>1
        disp('paila, D>1');
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
    j=dosr.jacob0(q_elip(:,i))
    j=[j(1,:);j(2,:)];
    c=dosr.fkine(q_elip(:,i));
    dosr.plot(q_elip(:,i)')
    plot_ellipse(j*j',c(1:2,4))
    pause(1)
end


%% Punto 6
% Para el manipulador tipo elbow, utilice una articulación dummy adicional para con-
% siderar una extensión de longitud l3 desde la tercera articulación hasta el efector final.
% * Determine el Jacobiano Geometrico por propagacion de velocidades.
% * Determine el Jacobiano Geometrico por derivada de la matriz de rotacion.
clear all
clc

syms q1 q2 q3 q4 q1p q2p q3p q4p l1 l2 l3 real
% l1 = 1;
% l2 = 1;
% l3 = 1;
L(1) = Link('revolute','alpha',0,'a',0,'d',l1,'offset',0,'modified');
L(2) = Link('revolute','alpha',pi/2,'a',0,'d',0,'offset',0,'modified');
L(3) = Link('revolute','alpha',0,'a',l2,'d',0,'offset',0,'modified');
L(4) = Link('revolute','alpha',0,'a',l3,'d',0,'offset',0,'modified');
tresR=SerialLink(L, 'name','tresR');
% 
% q1 = 0;
% q2 = 0;
% q3 = 0;
% q4 = 0;
% q =  [q1 q2 q3 q4];
% 
% figure (1)
% title('Manipulador en posicio inicial')
% trplot(eye(4),'length',1,'rgb');
% hold on
% tresR.plot(q,'workspace',[-10 10 -10 10 -10 10],'scale',0.1,'jaxes');
% axis([-5 5 -5 5 -5 5])
% tresR.teach();
%
% Cinemática directa
T01 = simplify(L(1).A(q1));
T12 = simplify(L(2).A(q2));
T12(2,1)=0;
T12(2,2)=0;
T12(3,3)=0;
T23 = simplify(L(3).A(q3));
T34 = simplify(L(4).A(q4));
T02 = simplify(T01*T12);
T03 = simplify(T01*T12*T23);
T04 = simplify(T01*T12*T23*T34);

% Jacobiana Geometrico 
% (Derivando matriz de Rotacion)
T = T04;

% Se determina el Jacobiano de posicion Jv
dx = T(1,4);  % componente x
dy = T(2,4);  % componente y
dz = T(3,4);  % componente z

% Derivando con respecto al tiempo
% Nota: Se aplica la regla de la cadena
vx = [diff(dx,q1), diff(dx,q2), diff(dx,q3), diff(dx,q4)]; 
vy = [diff(dy,q1), diff(dy,q2), diff(dy,q3), diff(dx,q4)];
vz = [diff(dz,q1), diff(dz,q2), diff(dz,q3), diff(dx,q4)];

% Se tiene el Jacobiano de posicion Jv 
Jv = [vx; vy; vz];

% Se determina el Jacobiano de Orientacion Jw
% Tomando la Matriz de rotacion de T
R = T(1:3,1:3);

% Derivando R se tiene
% 1. Las derivadas de q con respecto al tiempo simbolicas
syms dotq1 dotq2 dotq3 dotq4 real

% 2. La derivada de R resulta
dotR = simplify(   diff(R,q1) * dotq1 + diff(R,q2) * dotq2...
                 + diff(R,q3) * dotq3 + diff(R,q4) * dotq4);

% Se determina la matriz skew
Omega = dotR * R'; % Transpuesta de la matriz de rotaci?n
Omega = simplify(Omega);

% Logrando el vector w = [wx wy wz]'
wx = Omega(3,2);
wy = Omega(1,3);
wz = Omega(2,1);

% Organizando la Matriz jacobiana de orientacion Jw y sabiendo que las
% velocidades angular resultan de una combinacion lineal de ellas se tiene:
Jw1 = [wx wy wz]'; % Columnas para dotq1
Jw1=subs(Jw1,dotq1,1);
Jw1=subs(Jw1,dotq2,0);
Jw1=subs(Jw1,dotq3,0);
Jw1=subs(Jw1,dotq4,0);
Jw2 = [wx wy wz]'; % Columnas para dotq2
Jw2=subs(Jw2,dotq1,0);
Jw2=subs(Jw2,dotq2,1);
Jw2=subs(Jw2,dotq3,0);
Jw2=subs(Jw2,dotq4,0);
Jw3 = [wx wy wz]'; % Columnas para dotq3 
Jw3=subs(Jw3,dotq1,0);
Jw3=subs(Jw3,dotq2,0);
Jw3=subs(Jw3,dotq3,1);
Jw3=subs(Jw3,dotq4,0);
Jw4 = [wx wy wz]'; % Columnas para dotq3 
Jw4=subs(Jw4,dotq1,0);
Jw4=subs(Jw4,dotq2,0);
Jw4=subs(Jw4,dotq3,0);
Jw4=subs(Jw4,dotq4,1);
Jw = [Jw1 Jw2 Jw3 Jw4];

% Por ultimo organizando tanto el Jv y Jw se tiene
Jgeo_derR = [Jv; Jw]


% Jacobiano geometrico por propagacion de velocidades
% Calculo de los 0Zi 
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
z03 = T03(1:3,3); % Art 3
z04 = T04(1:3,3); % Art 4
% Calculo del vector ^iP_n
p44 = T04(1:3,4) - T04(1:3,4);
p34 = T04(1:3,4) - T03(1:3,4);
p23 = T04(1:3,4) - T02(1:3,4);
p13 = T04(1:3,4) - T01(1:3,4);

% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional 
J3 =simplify([(skew(z03) * p34);      z03]); % Rotacional 
J4 =simplify([(skew(z04) * p44);      z04]); % Rotacional
% Tambien funciona cross(z03,p34)
Jgeo = simplify([J1, J2, J3, J4])

% * En la configuración anterior calcule la matriz Jacobiana sin considerar la articulacion
% dummy, en caso de haber alguna diferencia ¿cual es el origen de esta?

% Jacobiano Simbolico
% Coordenadas generalizadas
pos_x = T04(1,4);
pos_y = T04(2,4);
pos_z = T04(3,4);
% Casillas del Jacobiano
J11 = diff(pos_x,q1);
J12 = diff(pos_x,q2);
J13 = diff(pos_x,q3);

J21 = diff(pos_y,q1);
J22 = diff(pos_y,q2);
J23 = diff(pos_y,q3);

J31 = diff(pos_z,q1);
J32 = diff(pos_z,q2);
J33 = diff(pos_z,q3);

Jsym_sin_dummy = [J11 J12 J13;
        J21 J22 J23;
        J31 J32 J33]
    
% Jacobiano geometrico
% Calculo de los 0Zi 
z01 = T01(1:3,3); % Art 1
z02 = T02(1:3,3); % Art 2
z03 = T03(1:3,3); % Art 3
% Calculo del vector ^iP_n
p34 = T04(1:3,4) - T03(1:3,4);
p23 = T04(1:3,4) - T02(1:3,4);
p13 = T04(1:3,4) - T01(1:3,4);

% Calculo de las columnas del jabobiano
J1 =simplify([(skew(z01) * p13);      z01]); % Rotacional
J2 =simplify([(skew(z02) * p23);      z02]); % Rotacional 
J3 =simplify([(skew(z03) * p34);      z03]); % Rotacional 

Jgeo_sin_dummy = simplify([J1, J2, J3])

% Se puede ver que a diferencia de los jacobianos que incluyen la
% articulacion dummy, este tiene 3 columnas, ya que al tenerla en cuenta,
% se puede observar como en esta solo se tiene velocidad angular ya que
% esta aritculacion no posee un brazo que le proporcione velociades
% lineales.

% * Considere la configuración q = [ pi/4 pi/4 pi/2 ], halle el valor numerico de los items anteriores,
%  compare con lo obtenido con la funcion jacob0.
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

Jgeo_derR =eval(Jgeo_derR)

j0 = (tres.jacob0([q1 q2 q3 q4]))

%% Punto 7
% Para el robot PPPR, utilice una articulación dummy de longitud l 1 = 1 m para
% representar el efecto final.
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
% 
% q1 = 1;
% q2 = 1;
% q3 = 1;
% q4 = 0;
% q5 = 0;
% q =  [q1 q2 q3 q4 q5];
% 
% figure (1)
% title('Manipulador en posicio inicial')
% trplot(eye(4),'length',1,'rgb');
% hold on
% PPPR.plot(q,'workspace',[-10 10 -10 10 -10 10],'scale',0.1,'jaxes');
% axis([-1 4 -1 4 -0 5])
% PPPR.teach()

% Cinemática directa
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

% * Dada la configuración [q1 q2 q3 q4 ] = [1 1 1 pi/2]
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

% * Dado el vector q punto= [1,5 1,2 1,8 pi/4 ]' encuentre Ẋ usando la configuración del item anterior.
% ¿Que significado tiene el vector de velocidades generalizados? (Explique su respuesta).

q_dot = [1.5 1.2 1.8 pi/4 0]';
X = Jgeo*q_dot

% se puede observar en la respuesta que tiene velocicdad en los 3 ejes
% cartesianos, al tener 1 articualicon prismatica en cada uno, y tendra una
% velocidad angular en el eje y, que es coo se encuentra dispuesta la
% cuarta articulacion, la cual es rotacional.
