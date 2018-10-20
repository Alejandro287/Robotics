%% Cinematica directa 6 DOF
% DHmod
clear all 
clc

escala = 1;
l = escala*[250 300 300 70];
qlims = deg2rad([-180 180; -90 110; -230 50; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(1),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(2),'d',0,   'offset',-pi/2,'qlim',qlims(3,:),'modified');   
L(4) = Link('revolute','alpha',-pi/2, 'a',0,   'd',l(3),'offset', 0,   'qlim',qlims(4,:),'modified');
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(5,:),'modified');
L(6) = Link('revolute','alpha',-pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(6,:),'modified');

R6 = SerialLink(L,'name','6 DOF');
R6.tool= transl(0,0,l(4));
%%
q = [0 -pi/12 -pi/6 pi/12 pi/6 pi/6]
q0 = zeros(1,6);
trplot(eye(4),'length',200,'rgb','frame','0');
hold on
R6.plot(q,'workspace',escala*[-1000 1000 -1000 1000 -1000 1000],'tilesize',100,'scale',0.4);
axis(escala*[-1000 1000 -1000 1000 -1000 1000]);
R6.teach()
M = eye(4); 
for i=1:R6.n
    M = M * L(i).A(q(i));
    if i == 4 || i == 3
      trplot(M,'rgb','frame',num2str(i),'length',200)
    end
end
%%
% Inversa
Tt = R6.fkine(q);
Posw =  Tt(1:3,4) - l(4)*Tt(1:3,3);
Tw = Tt;
Tw(1:3,4) = Posw;
%%
% Posicion
q1a = atan2(Tw(2,4), Tw(1,4));
q1b = atan2(-Tw(2,4),-Tw(1,4));

pxy = sqrt(Tw(1,4)^2 + Tw(2,4)^2);
z = Tw(3,4) - l(1);
r = sqrt(pxy^2 + z^2);

the3 = acos((r^2 - l(2)^2 -l(3)^2)/(2*l(2)*l(3)));
alp = atan2(z,pxy);

the2a = alp - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
the2b = alp + atan2(l(3)*sin (the3),l(2)+l(3)*cos(the3));
q2a = -the2a; 
q3a = -the3;

q2b = -(the2b);
q3b = the3;
disp('q_pos')
disp([q1a q2a q3a])
disp([q1b q2b q3b])

%%
% Orientacion
T03 = R6.A([1 2 3],[q1a q2a q3a])*transl(0,l(3),0);
R30 = T03(1:3,1:3)';
R46 = R30*Tw(1:3,1:3);
tr2eul(R46)
%%
syms q4 q5 q6
T34 = trotx(-pi/2)*trotz(q4);
T34(2,1) = 0;
T34(2,2) = 0;
T34(3,3) = 0;
T45 = trotx(pi/2)*trotz(q5);
T45(2,1) = 0;
T45(2,2) = 0;
T45(3,3) = 0;
T56 = trotx(-pi/2)*trotz(q6);
T56(2,1) = 0;
T56(2,2) = 0;
T56(3,3) = 0;
T46 = simplify(T34*T45*T56)
%%
% Solucion Muñeca
q5a = atan2(sqrt(1-R46(2,3)^2),R46(2,3));
q5b = atan2(-sqrt(1-R46(2,3)^2),R46(2,3));
q4 = atan2(R46(3,3),-R46(1,3));
q6 = atan2(-R46(2,2),R46(2,1));
disp('q_ori')
disp([q4 q5a q6])
disp([q4 q5b q6])