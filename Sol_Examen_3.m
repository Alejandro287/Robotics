% Examen 3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Con teach target -> Grabando configuracion del robot
% 2. Creando target con posicion y orientacion
% 3. Creando target a partir de geometria importada
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Punto 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
escala = 1;
l = escala*[70 352 360 380];
qlims = deg2rad([-180 180; -90 110; -230 50]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(2),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',0,   'offset',-pi/2,'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(3),'d',0,   'offset', 0,   'qlim',qlims(3,:),'modified');

IRBpos = SerialLink(L,'name','IRBPos');
IRBpos.tool= transl(0,l(4),0)*trotx(-pi/2);
%%
qt = [pi/6 pi/12 -pi/24];
trplot(eye(4),'length',300,'rgb','frame','0');
hold on
IRBpos.plot(qt,'workspace',escala*[-1000 1000 -1000 1000 -1000 1000],'tilesize',100,'scale',0.35,'noa');
axis(escala*[-1000 1000 -1000 1000 -100 1000]);
%%
Tt = IRBpos.fkine(qt)

% Posicion del efector
Tw = Tt;

% Solucion para q1
q1a = atan2(Tw(2,4), Tw(1,4));
q1b = atan2(-Tw(2,4),-Tw(1,4));

% Plano articulaciones 2 - 3
pxy = sqrt(Tw(1,4)^2 + Tw(2,4)^2) - l(1); % Correccion de desplazamiento L2
z = Tw(3,4) - l(2); % Correccion por altura L1
r = sqrt(pxy^2 + z^2);

% Mecanismo 2R
the3 = acos((r^2 - l(3)^2 -l(4)^2)/(2*l(3)*l(4)));
alp = atan2(z,pxy);
the2a = alp - atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));
the2b = alp + atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));

% Tener en cuenta ajustes por desfases
% Codo Abajo
q2a =  pi/2 - the2a;
q3a = -(pi/2 + the3);

% Codo Arriba
q2b =  pi/2 - the2b;
q3b = -(pi/2 - the3);

% Soluciones
disp('Codo arriba')
disp(rad2deg([q1a q2b q3b]))

disp('Codo abajo')
disp(rad2deg([q1a q2a q3a]))

syms nx ny nz ox oy oz ax ay az px py pz
MTH = [nx ox ax px;
       ny oy ay py;
       nz oz az pz;
       0  0  0  1]
%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bonos
% Metodo de Pieper
% Diferencia en velocidades articulares
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
