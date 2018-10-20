
List = cell(1,11)

imp = [0,1,0,1,0,1,1,1,0,1,1]

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
          0,-25,20,0,70;]
      
for i=1:11
    if imp(i) == 0
        List{1,i} = puntos(i,:);
    else
        List{1,i} = trayectoria (puntos(i-1,:), puntos(i,:));
    end
end
List{1,6}{1,10}
List{1,6}{1,10}(2)
length(List{1,6}{1,10})

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
    pos
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


% while 1
%       
%     prompt = ('Vector de [x,y,z,phi,Gripper]:');
%     T = input(prompt);
%     
%     for i=1:Phantom_X.n
%         delete(Mp(i))
%     end
%     
%     %% Envíar mensaje
%     
%     T(4) = (T(4).*(pi/180));
%     T(5) = (T(5).*(pi/180));
%     
%     
%     [Q1,Q2]=InvKin(T(1),T(2),T(3),T(4))
%     
%     baseMsg.Data = Q2(1)+2.618; % Valor del mensaje
%     send(basePub,baseMsg); % Envio
% 
%     motor2Msg.Data = Q2(2)+1.0379; % Valor del mensaje
%     send(motor2Pub,motor2Msg); % Envio
% 
%     motor3Msg.Data = Q2(3)+2.618; % Valor del mensaje
%     send(motor3Pub,motor3Msg); % Envio
% 
%     motor4Msg.Data = Q2(4)+2.618; % Valor del mensaje
%     send(motor4Pub,motor4Msg); % Envio
% 
%     gripperMsg.Data = T(5); % Valor del mensaje
%     send(gripperPub,gripperMsg); % Envio
%     
%     b1=0;
%     b2=0;
%         
%     while 1
%         %% Consulta de la posición
%         
%         base   = receive(baseSub); 
%         motor2 = receive(motor2Sub); 
%         motor3 = receive(motor3Sub); 
%         motor4 = receive(motor4Sub); 
%         gripper= receive(gripperSub); 
%         
%         baseAngle   = base.CurrentPos-2.618; % Angulo de la base
%         motor2Angle = motor2.CurrentPos-1.0379; % Angulo del motor 2
%         motor3Angle = motor3.CurrentPos-2.618; % Angulo del motor 3
%         motor4Angle = motor4.CurrentPos-2.618; % Angulo del motor 4
%         gripperAngle= gripper.CurrentPos; % Angulo del gripper
%         
%         %% Grafica
%         trplot(eye(4),'length',2,'rgb');
%         hold on
%         Phantom_X.plot([baseAngle motor2Angle motor3Angle motor4Angle],'workspace',[-40 50 -40 50 -40 50],'scale',0.3,'jaxes');
%         %
%         % Workspace necesario para robots con juntas P
%         % scale Robot refinado
%         % jaxes Ejes de articulación
%         axis([-40 50 -40 50 -40 50])
%         
%         b1 = abs(base.Error)+abs(motor2.Error)+abs(motor3.Error)+abs(motor4.Error)+abs(gripper.Error);
%         
%         if b1 == b2
%             
%             q_alt = [baseAngle motor2Angle motor3Angle motor4Angle];
%             M = eye(4);
%             Mp = ones(1, Phantom_X.n);
%             for i=1:Phantom_X.n
%                 M = M * L(i).A(q_alt(i));
%                 Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',4,'text_opts', {'FontSize', 5});
%             end
%             
%             break
%         end
%         
%         b2 = b1;
%         
%     end
% end



function [Q1,Q2] = InvKin(T)
    l1 = 14.04;
    l2 = 10.67;
    l3 = 10.67;
    l4 = 10.53;
    
    xff=T(1,4);
    yff=T(2,4);
    zff=T(3,4);
    eul=tr2eul(T);
    phi=eul(1,2);
    

      
    zf=zff-l1
    q1=atan2(yff,xff)
    xf=sqrt((yff^2)+(xff^2))
        
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