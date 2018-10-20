

%%
%Punto 2

clear all      
clc

q1=0;
q2=0;
 
P2=[0    0      q1    0    0    1;
    0    pi/2   q2    0    0    1];


[Robot,L,A0N] = ForKin(P2);

fprintf('\n\n Evaluacion de A0N con vector  de variables Q = [q1 q2 ... qn] \n')
Q = zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q, 3, 3, 7)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()


%%
%Punto 3

clear all 
clc

q1=0;
q2=0;
q3=0;

P3=[0    0       0     q1    0    0;
    0    pi/2    q2    0     0    1;
    0    pi/2    0     q3    0    0];


[Robot,L,A0N] = ForKin(P3);

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q = zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q, 3, 3, 7)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()

%%
%Punto 4

clear all 
clc

q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
 
P4=[0           0      1          q1    0      0;
    0           pi/2   q2         0     1      1;
    0           pi/2   1          q3    pi/2   0;
    0          -pi/2   0          q4    pi/4   0;
    sqrt(2)/2   pi/2   sqrt(2)/2  q5   -pi/2   0;
    0          -pi/2   sqrt(2)    0     0     -1];


[Robot,L,A0N] = ForKin(P4);

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q = zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q, 5, 3, 8)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()


%%
%Punto 5

clear all 
clc

syms L1 L2 L3 L4 as real 
syms q1 q2 q3 q4 q5 q6 as real 


P5=[0    0      L1     q1   0     0;
    0    pi/2   0      q2   0     0;
    L2   0      0      q3   pi/2  0;
    0    pi/2   L3     q4   pi/2  0;
    0    pi/2   0      q5   0     0;
    0   -pi/2   0      q6   0     0;
    0    0      L4     0    0    -1];


[Robot,L,A0N] = ForKin(P5);

q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;

L1=40;
L2=30;
L3=50;
L4=30;

P5=eval(P5);

[Robot,L,A0N] = ForKin(P5);

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q= zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q, 5, 20, 10)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q=[pi/6, -pi/6, -pi/12, -pi/2, pi, pi/2]
evaluar(A0N, Q);
graficar (Robot,L, Q, 5, 20, 10)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()



%%
%Punto 6

clear all 
clc

syms L1 L2 L3 L4 L5 as real 
syms q1 q2 q3 q4 q5 q6 as real 

P6=[0    0      L1     q1    0     0;
    0   -pi/2   L2     q2   -pi/2  0;
    0   -pi/2   L3     q3    pi/2  0;
    L2   pi/2   0      q4    pi/2  0;
    L4   pi/2   q5     pi/2  L5    1;
    0    pi/2   0      q6    0     0;];

[Robot,L,A0N] = ForKin(P6);

q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;

L1=10;
L2=5;
L3=10;
L4=5;
L5=5;

P6=eval(P6);

[Robot,L,A0N] = ForKin(P6);

[N,~]=size(P6);

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q=zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q , 0, 20,10)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause ()


%%
%Punto 7

clear all 
clc

syms L1 L2 L3 L6 as real 
syms q1 q2 q3 q4 q5 q6 as real 

P7=[0    0      L1     q1    0     0;
    0   -pi/2   L2     q2   -pi/2  0;
    0    pi/2   q3     pi/2  L3    1;
    0    0      0      q4    0     0;
    0    pi/2   0      q5    0     0;
    0   -pi/2   0      q6    0     0;
    0    0      L6     0     0    -1];

 
[Robot,L,A0N] = ForKin(P7);

L1=20;
L2=15;
L3=0;
L6=20;


q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;

P7=eval(P7);

[Robot,L,A0N] = ForKin(P7);

[N,~]=size(P7);


fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q= zeros(1,Robot.n)
evaluar(A0N, Q);
graficar (Robot,L, Q , 30, 20, 10)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause (5)

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q=[pi/6, pi/12, 20, 0, pi, pi/2]
evaluar(A0N, Q);
graficar (Robot,L, Q, 30, 20, 10)

disp('PULSA CUALQUIER TECLA PARA CONTINUAR');
pause (5)

fprintf('\n\n Evaluacion de A0N con vector de variables Q = [q1 q2 ... qn] \n')
Q=[pi/6, -pi/12, 35, 0, 0, pi/2]
evaluar(A0N, Q);
graficar (Robot,L, Q, 30, 20, 10)


%%
%

function [Rob,l,T] = ForKin(DHpar)

    [N,~] = size(DHpar);
    proof=0;
    
    for i = 1 : N
        
        if DHpar(i,6) == 0
            L(i) = Link('revolute','alpha',DHpar(i,2),'a',DHpar(i,1),'d',DHpar(i,3),'offset',DHpar(i,5),'qlim',[-pi pi],'modified');
        elseif DHpar(i,6) == 1
            L(i) = Link('prismatic','alpha',DHpar(i,2),'a',DHpar(i,1),'theta',DHpar(i,4),'offset',DHpar(i,5),'qlim',[0, 40],'modified');
        else
            Robot = SerialLink(L,'name','Robot');
            
            Twt = trotx(DHpar(i,2))*transl(DHpar(i,1),0,0)*...
                  trotz(DHpar(i,4))*transl(0,0,DHpar(i,3));
              
            Twt(isAlways(abs(Twt) <= 1e-10,'Unknown','false') ) = 0;
            
            Robot.tool = trotx(DHpar(i,2))*transl(DHpar(i,1),0,0)*...
                         trotz(DHpar(i,4))*transl(0,0,DHpar(i,3));
            
            proof=1;
            break;
        end
    end
    
    if proof==0
        Robot = SerialLink(L,'name','Robot');
    end
    
    Robot
    
    Rob = Robot;
    
    l=L;
    
    d1 = digits(5);
    eval(sprintf('A%d%d = eye(4);', 0, N));
    
    for i = 1 : N

        if i==N && proof==1
            eval(sprintf('A%d%d = vpa(simplify(A%d%d*Twt));', 0, N, 0, N));
            break
        end
        eval(sprintf('syms q%d as real', i));
        eval(sprintf('A%d%d = vpa(simplify(L(i).A(q%d)));', i-1, i, i));
        eval(sprintf('A%d%d(isAlways(abs(A%d%d) <= 1e-10,''Unknown'',''false'')) = 0;'...
                       , i-1, i, i-1, i))
        if L(i).sigma
            eval(sprintf('Ainter = A%d%d;', i-1, i));
            
            eval(sprintf('q%d = 1;', i));
            
            symb = symvar(Ainter);
            
            for j=1 : length(symb)
                eval(strcat(char(symb(1,j)),'=1;'))
            end
            
            Ainter = round(eval(Ainter), 6, 'significant');
            
            for j=1 : 4
                for k=1 : 4
                    if abs(Ainter(j,k)) <= 1e-10
                        Ainter(j,k) = 0;
                    else
                        Ainter(j,k) = 1;
                    end
                end
            end
            
            eval(sprintf('A%d%d = vpa(simplify(Ainter.*A%d%d));', i-1, i, i-1, i));
            eval(sprintf('syms q%d as real', i));
            
            for j=1 : length(symb)
                eval(['syms ', char(symb(1,j)),' as real'])
            end
        end
                
        eval(sprintf('A%d%d', i-1, i))
        
        eval(sprintf('A%d%d = vpa(simplify(A%d%d*A%d%d));', 0, N, 0, N, i-1, i));
    end
    
    if proof
        eval(sprintf('A%d%d = vpa(Twt);', N-1, N));
        eval(sprintf('A%d%d', N-1, N))
    end
    
    eval(sprintf('T = vpa(simplify(A%d%d));', 0, N))
    eval(sprintf('A%d%d', 0, N))
    
    %fkine(Robot)
    
    fprintf('\n\nPosición en coordenadas Cartesianas: Pos = [x y z]''\n')
    
    Pos = vpa([T(1,4); T(2,4); T(3,4)])
    
    fprintf('\n\nOrientación en angulos fijos tipo XYZ: fijXYZ = [alpha beta gamma]\n')
    
    r = T(1:3,1:3);
    
    r31 = r(3,1)
    r11 = r(1,1)
    r21 = r(2,1)
    r32 = r(3,2)
    r33 = r(3,3)
    
    fprintf('beta = atan2(-r31,sqrt((r11^2)+(r(21^2))))\n\n')
    fprintf('alpha = atan2(r32/cos(B),r33/cos(B))\n\n')
    fprintf('gamma = atan2(r21/cos(B),r11/cos(B))\n\n')

    
    fprintf('\n\nOrientacion en angulos de Euler tipo XYZ: eulXYZ = [alpha beta gamma]\n')
    
    r = T(1:3,1:3);
    
    r13 = r(1,3)
    r23 = r(2,3)
    r33 = r(3,3)
    r12 = r(1,2)
    r11 = r(1,1)
    
    fprintf('beta = atan2(r13,sqrt((r23^2)+(r(33^2))))\n\n')
    fprintf('alpha = atan2(-r23/cos(B),r33/cos(B))\n\n')
    fprintf('gamma = atan2(-r12/cos(B),r11/cos(B))\n\n')
    
        
end


function T = evaluar(A0N, Q)
    
    for i=1 : length(Q)
        eval(sprintf('q%d = Q(i);', i));
    end
    
    T = eval(A0N)
    
    fprintf('\n\nPosición en coordenadas Cartesianas: Pos = [x y z]\n')
    
    Pos = vpa([T(1,4), T(2,4), T(3,4)])
    
    fprintf('\n\nMatriz de rotación:\n')
    
    r = T(1:3,1:3)
    
    fprintf('\n\nOrientacion en angulos fijos tipo XYZ: fijXYZ = [alpha beta gamma]\n')
    fijXYZ = ones(1,3);
    fijXYZ(2) = atan2(-r(3,1),sqrt((r(1,1)^2)+(r(2,1)^2)));
    
    if fijXYZ(2) == pi/2
        fijXYZ(1) = 0;
        fijXYZ(3) = atan2(r(1,2),r(2,2));
    elseif fijXYZ(2) == -pi/2
        fijXYZ(1) = 0;
        fijXYZ(3) = -atan2(r(1,2),r(2,2));
    else
        fijXYZ(1) = atan2(r(3,2)/cos(fijXYZ(2)),r(3,3)/cos(fijXYZ(2)));    
        fijXYZ(3) = atan2(r(2,1)/cos(fijXYZ(2)),r(1,1)/cos(fijXYZ(2)));
    end
    
    disp(fijXYZ)
    
    
    fprintf('\n\nOrientacion en angulos de Euler tipo XYZ: eulXYZ = [alpha beta gamma]\n')
    
    eulXYZ = ones(1,3);
    eulXYZ(2) = atan2(r(1,3),sqrt((r(2,3)^2)+(r(3,3)^2)));
    
    if eulXYZ(2) == pi/2
        eulXYZ(1) = 0;
        eulXYZ(3) = atan2(r(2,1),r(2,2));
    elseif eulXYZ(2) == -pi/2
        eulXYZ(1) = 0;
        eulXYZ(3) = atan2(r(2,1),r(2,2));
    else
        eulXYZ(1) = atan2(-r(2,3)/cos(eulXYZ(2)),r(3,3)/cos(eulXYZ(2)));    
        eulXYZ(3) = atan2(-r(1,2)/cos(eulXYZ(2)),r(1,1)/cos(eulXYZ(2)));
    end
    
    disp(eulXYZ)
    
    
    
    fprintf('\n\nOrientacion en angulos de Euler tipo ZYZ: eulZYZ = [alpha beta gamma]\n')

    eulZYZ = ones(1,3);
    eulZYZ(2) = atan2(sqrt((r(3,1)^2)+(r(3,2)^2)),r(3,3));
    
    if eulZYZ(2) == 0
        eulZYZ(1) = 0;
        eulZYZ(3) = atan2(-r(1,2),r(1,1));
    elseif abs(eulZYZ(2)) == pi || abs(eulZYZ(2)) == -pi 
        eulZYZ(1) = 0;
        eulZYZ(3) = atan2(r(1,2),-r(1,1));
    else
        eulZYZ(1) = atan2(r(2,3)/sin(eulZYZ(2)),r(1,3)/sin(eulZYZ(2)));    
        eulZYZ(3) = atan2(r(3,2)/sin(eulZYZ(2)),-r(3,1)/sin(eulZYZ(2)));
    end 
    
    disp(eulZYZ)
    
end


function graficar (Robot, L, Q, escala1, escala2, escala3)
    
    cla();
    

    tam = (sum(Robot.d)+sum(Robot.a))+escala1;

    trplot(eye(4),'length',2,'rgb');
    hold on
    axis([-tam tam -tam tam -tam tam])
    Robot.plot(Q,'workspace',[-tam tam -tam tam -tam tam],'scale',0.25);
    %Robot.teach()
    
    q_alt = Q;
    M = eye(4);
    Mp = ones(1, Robot.n);
    for i=1:Robot.n
        M = M * L(i).A(q_alt(i));
        Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',escala2,'text_opts', {'FontSize', escala3});
    end


end


