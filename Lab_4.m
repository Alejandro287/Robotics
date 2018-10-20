%% Laboratorio No. 3 - Análisis.
%% Realizado por:
% Manuel Bejarano, Alejandro Cano y Julio Cerón.
%% Definición de eslabones
clear all 
clc

l1 = 14.04;
l2 = 10.67;
l3 = 10.67;
%l4 = 7.249;
l4 = 10.53;

L(1) = Link('revolute', 'alpha',    0, 'a',  0, 'd', l1, 'offset',    0, 'qlim', [-2.62 2.61], 'modified');
L(2) = Link('revolute', 'alpha', pi/2, 'a',  0, 'd',  0, 'offset',    0, 'qlim', [(-0.673) (3.814)], 'modified');
L(3) = Link('revolute', 'alpha',    0, 'a', l2, 'd',  0, 'offset',    0, 'qlim', [(-2.79) (2.79)], 'modified');
L(4) = Link('revolute', 'alpha',    0, 'a', l3, 'd',  0, 'offset', pi/2, 'qlim', [(-2.79) (2.79)], 'modified');

%% Conexión de eslabones
Phantom_X = SerialLink(L,'name','Phantom_X');
Phantom_X.tool = trotx(pi/2)*transl([0,0,0])*trotz(0)*transl([0,0,l4]);
%%
 
% [Q1,Q2]=InvKin(0,0,0,0)
%    
% graficar(Phantom_X, L, Q2, 5)

List = cell(1,11);

imp = [0,1,0,1,0,1,1,1,0,1,1];

puntos = [0,25,20,0,150;
          0,25,8,0,150;
          0,25,8,0,70;
          0,25,20,0,70;
          25,0,20,0,70;
          30,0,20,0,70;
          30,0,8,0,70;
          30,0,20,0,70;
          0,-25,20,0,70;
          0,-25,8,0,70;
          0,-25,20,0,70;];
      
for i=1:11
    if imp(i) == 0
        List{1,i} = puntos(i,:);
    else
        List{1,i} = trayectoria (puntos(i-1,:), puntos(i,:));
    end
end


for i=1:length(List)
    if length(List{1,i}) == 5
        [Q1,Q2]=InvKin(List{1,i}(1),List{1,i}(2),List{1,i}(3),List{1,i}(4));
        graficar(Phantom_X, L, Q2, 5)
    else
        for j=1:length(List{1,i})
            [Q1,Q2]=InvKin(List{1,i}{1,j}(1),List{1,i}{1,j}(2),List{1,i}{1,j}(3),List{1,i}{1,j}(4));
            graficar(Phantom_X, L, Q2, 5)
        end
    end
    
end



function T = trayectoria (punto1, punto2)
    proof = punto2-punto1
    pos = 0;
    if proof(1) ~= 0 
        pos=1;
    elseif proof(2) ~=0
        pos=2;
    elseif proof(3) ~=0
        pos=3;
    elseif proof(4) ~=0
        pos=4;
    end
    paso = 0.5; 
    
    tam = abs(proof(pos))/paso
    if proof(pos) < 0
       paso = -paso;
    end
    T = cell(1,tam);
    punto = punto1;
    for i=1:tam
        punto(pos) = punto1(pos)+(paso*i);
        T{1,i} = punto;
    end   
end

%pause ()
% qready = [0 0 0 0]; % initial position of robot
% Td = transl([5 10 15]);
% plot(Phantom_X,qready,'noname')
% q = Phantom_X.ikine(Td, qready,[1 1 1 0 1 0],'ilimit', 10000, 'plot')
% graficar(Phantom_X, L, q, 5)
% plot(Phantom_X,q,'noname');
% Phantom_X.teach()

%%
function [Q1,Q2] = InvKin(xff,yff,zff,phi)
    l1 = 14.04;
    l2 = 10.67;
    l3 = 10.67;
    %l4 = 7.249;
    l4 = 10.53;

    
    
    zf=zff-l1
    q1=atan2(yff,xff)
    xf=sqrt((yff^2)+(xff^2))
    %xf=xff/cos(q1)
        
    x0=xf-l4*cos(phi)
    z0=zf-l4*sin(phi)
    D=(x0^2+z0^2-l2^2-l3^2)/(2*l2*l3)
    if abs(D)>1
        disp('paila, D>1');
        Q1=NaN;
        Q2=NaN;
    else
        q3_1=atan2(sqrt(1-D^2),D);
        q3_2=atan2(-sqrt(1-D^2),D);
        q2_1=atan2(z0,x0)-atan2(l3*sin(q3_1),l2+l3*cos(q3_1));
        q2_2=atan2(z0,x0)-atan2(l3*sin(q3_2),l2+l3*cos(q3_2));
        q4_1=phi-q2_1-q3_1;
        q4_2=phi-q2_2-q3_2;
        Q1=[q1,q2_1,q3_1,q4_1];
        Q2=[q1,q2_2,q3_2,q4_2];
    end
end

function graficar (Robot, L, Q, escala)
    
    cla();

    tam = (sum(Robot.d)+sum(Robot.a))+escala;

    trplot(eye(4),'length',2,'rgb');
    hold on
    axis([-tam tam -tam tam -tam tam])
    Robot.plot(Q,'workspace',[-tam tam -tam tam -tam tam],'scale',0.3);
    Robot.teach()
    
%     q_alt = Q;
%     M = eye(4);
%     Mp = ones(1, Robot.n);
%     for i=1:Robot.n
%         M = M * L(i).A(q_alt(i));
%         Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',5,'text_opts', {'FontSize', 10});
%     end


end