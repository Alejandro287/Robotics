% clear all 
% clc
% 
% q1=0;
% q2=0;
% q3=0;
% 
% l1 = 3;
% l2 = 1;
% 
% dh=[0    0      l1     q1   0       0;
%     0    pi/2   0      q2   pi/2    0;
%     0    pi/2   q3     0    1       1;
%     0    0      l2     0    0      -1;];
% 
% 
% 
% A0N=ForKin(dh)
% Q=[0,0,0]';
% A0N=evaluar(A0N, Q)
% 
% %%
% %Punto 2
% 
% clear all 
% clc
% 
% q1=0;
% q2=0;
% 
% P2=[0    0      q1    0    0    1;
%     0    pi/2   q2    0    0    1];
% 
%     
% A0N=ForKin(P2)
% Q=[0,0]';
% A0N=evaluar(A0N, Q)


%%
%Punto 3

% clear all 
% clc
% 
% q1=0;
% q2=0;
% q3=0;
% 
% P3=[0    0      0     q1   0    0;
%     0    pi/2   q2    0    0    1;
%     0    pi/2   0     q3    0    0];
% 
%     
% A0N=ForKin(P3)
% Q=[0,0,0]';
% A0N=evaluar(A0N, Q)

%%
%Punto 4

% clear all 
% clc
% 
% q1=0;
% q2=0;
% q3=0;
% q4=0;
% q5=0;
% 
% P4=[0    0      1     q1   0    0;
%     0    pi/2   q2    0    1    1;
%     0    pi/2   1     q3   pi/2    0;
%     0   -pi/2   0     q4   pi/4  0;
%     sqrt(2)/2 pi/2 sqrt(2)/2 q5 -pi/2 0;
%     0 -pi/2 sqrt(2) 0 0 -1];
% 
%     
% A0N=ForKin(P4)
%[N,~]=size(P4);
%Q=zeros(1,N-1)
% A0N=evaluar(A0N, Q)


%%
%Punto 5

% clear all 
% clc
% 
% q1=0;
% q2=0;
% q3=0;
% q4=0;
% q5=0;
% q6=0;
% 
% L1=40;
% L2=30;
% L3=50;
% L4=30;
% 
% P5=[0    0      L1     q1   0     0;
%     0    pi/2   0      q2   0     0;
%     L2   0      0      q3   pi/2  0;
%     0    pi/2   L3     q4   pi/2  0;
%     0    pi/2   0      q5   0     0;
%     0   -pi/2   0      q6   0     0;
%     0    0      L4     0    0    -1];
% 
%  
% A0N=ForKin(P5)
% [N,~]=size(P5);
% Q=zeros(1,N-1);
% A0N=evaluar(A0N, Q)

%%
%Punto 6

% clear all 
% clc
% 
% q1=0;
% q2=0;
% q3=0;
% q4=0;
% q5=0;
% q6=0;
% 
% L1=10;
% L2=5;
% L3=10;
% L4=5;
% L5=5;
% Lt=2;
% 
% P6=[0    0      L1     q1    0     0;
%     0   -pi/2   L2     q2   -pi/2  0;
%     0   -pi/2   L3     q3    pi/2  0;
%     L2   pi/2   0      q4    pi/2  0;
%     L4   pi/2   q5     pi/2  L5    1;
%     0    pi/2   0      q6    0     0;
%     0   -pi/2   Lt     0     0    -1];
% 
%  
% A0N=ForKin(P6)
% [N,~]=size(P6);
% Q=zeros(1,N-1);
% A0N=evaluar(A0N, Q)

%%
%Punto 7

clear all 
clc

q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;

L1=20;
L2=15;
L3=0;
L6=20;

P7=[0    0      L1     q1    0     0;
    0   -pi/2   L2     q2   -pi/2  0;
    0    pi/2   q3     pi/2  L3    1;
    0    0      q4     0     0     1;
    0    pi/2   0      q5    0     0;
    0   -pi/2   q6     0     0     1;
    0    0      L6     0     0    -1];

 
A0N=ForKin(P7)
[N,~]=size(P7);
Q=zeros(1,N-1);
A0N=evaluar(A0N, Q)



function T = ForKin(DHpar)

    [N,~] = size(DHpar);
    proof=0;
    
    
    for i = 1 : N
        
        if DHpar(i,6) == 0
            L(i) = Link('revolute','alpha',DHpar(i,2),'a',DHpar(i,1),'d',DHpar(i,3),'offset',DHpar(i,5),'qlim',[-pi pi],'modified');
        elseif DHpar(i,6) == 1
            L(i) = Link('prismatic','alpha',DHpar(i,2),'a',DHpar(i,1),'theta',DHpar(i,4),'offset',DHpar(i,5),'qlim',[0, 10],'modified');
        else
            Robot = SerialLink(L,'name','Robot');
            
            Twt = trotx(DHpar(i,2))*transl(DHpar(i,1),0,0)*...
                  trotz(DHpar(i,4))*transl(0,0,DHpar(i,3));
              
            Twt(isAlways(Twt <= 1e-10,'Unknown','false') ) = 0;
            
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
    
    tam = (sum(Robot.d)+sum(Robot.a))+3;

    trplot(eye(4),'length',2,'rgb');
    hold on
    axis([-tam tam -tam tam -tam tam])
    Robot.plot(zeros(1,length(Robot.links)),'workspace',[-tam tam -tam tam -tam tam],'scale',0.3,'jaxes');
    Robot.teach()
    
%     q_alt = [0 0 0 0 0];
%     M = eye(4);
%     Mp = ones(1, Robot.n);
%     for i=1:Robot.n
%         M = M * L(i).A(q_alt(i));
%         Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',7,'text_opts', {'FontSize', 10});
%     end

    d1 = digits(5);
    eval(sprintf('A%d%d = eye(4);', 0, N));
    
    for i = 1 : N

        if i==N && proof==1
            eval(sprintf('A%d%d = vpa(simplify(A%d%d*Twt));', 0, N, 0, N));
            break
        end
        eval(sprintf('syms q%d as real', i));
        eval(sprintf('A%d%d = vpa(simplify(L(i).A(q%d)));', i-1, i, i));
        eval(sprintf('A%d%d(isAlways(A%d%d <= 1e-10,''Unknown'',''false'')) = 0'...
            , i-1, i, i-1, i))     
        eval(sprintf('A%d%d = vpa(simplify(A%d%d*A%d%d));', 0, N, 0, N, i-1, i));
    end
    
    eval(sprintf('T = vpa(A%d%d);', 0, N))
    
    disp('Posición en coordenadas Cartesianas: Pos = [x y z]''')
    
    Pos=vpa([T(1,4); T(2,4) ;T(3,4)])
    
    disp('Orientacion en angulos fijos tipo XYZ: eulXYZ = [x y z]''')
    
    r = T(1:3,1:3);
    
    r31 = r(3,1)
    r11 = r(1,1)
    r21 = r(2,1)
    r32 = r(3,2)
    r33 = r(3,3)
    
    disp('B = atan2(-r31,sqrt((r11^2)+(r(21^2))))')
    disp('alph = atan2(r21/cos(B),r11/cos(B))')
    disp('gam = atan2(r32/cos(B),r33/cos(B))')

    
    disp('Orientacion en angulos de Euler tipo XYZ: eulXYZ = [x y z]''')
    
    r = T(1:3,1:3);
    
    r13 = r(1,3)
    r23 = r(2,3)
    r33 = r(3,3)
    r12 = r(1,2)
    r11 = r(1,1)
    
    disp('B = atan2(r13,sqrt((r23^2)+(r(33^2))))')
    disp('alph = atan2(-r23/cos(B),r33/cos(B))')
    disp('gam = atan2(-r12/cos(B),r11/cos(B))')
end


function T = evaluar(A0N, Q)
    for i=1 : length(Q)
        eval(sprintf('q%d = Q(i);', i));
    end
    
    T = eval(A0N)
    
    disp('Posición en coordenadas Cartesianas: Pos = [x y z]''')
    
    Pos=vpa([T(1,4); T(2,4) ;T(3,4)])
    
    r = T(1:3,1:3);
    
    disp('Orientacion en angulos fijos tipo XYZ: eulXYZ = [x y z]''')
    
        
    B = atan2(-r(3,1),sqrt((r(1,1)^2)+(r(2,1)^2)))
    alph = atan2(r(2,1)/cos(B),r(1,1)/cos(B))
    gam = atan2(r(3,2)/cos(B),r(3,3)/cos(B))
    
    disp('Orientacion en angulos de Euler tipo XYZ: eulXYZ = [x y z]''')
    
 
    B = atan2(r(1,3),sqrt((r(2,3)^2)+(r(3,3)^2)))
    alph = atan2(-r(2,3)/cos(B),r(3,3)/cos(B))
    gam = atan2(-r(1,2)/cos(B),r(1,1)/cos(B))
end


