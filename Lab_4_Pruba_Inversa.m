%% Definición de eslabones
clear all 
clc

%4.029
%1.0379

l1 = 14.04;
l2 = 10.67;
l3 = 10.67;
%l4 = 7.249;
l4 = 10.53;

L(1) = Link('revolute', 'alpha',    0, 'a',  0, 'd', l1, 'offset',    0, 'qlim', [-2.618 2.618], 'modified');
L(2) = Link('revolute', 'alpha', pi/2, 'a',  0, 'd',  0, 'offset',    0, 'qlim', [-(1.82-(pi/2)) (1.78+(pi/2))], 'modified');
L(3) = Link('revolute', 'alpha',    0, 'a', l2, 'd',  0, 'offset',    0, 'qlim', [-pi pi], 'modified');
L(4) = Link('revolute', 'alpha',    0, 'a', l3, 'd',  0, 'offset', pi/2, 'qlim', [-pi pi], 'modified');

%% Conexión de eslabones
Phantom_X = SerialLink(L,'name','Phantom_X');
Phantom_X.tool = trotx(pi/2)*transl([0,0,0])*trotz(0)*transl([0,0,l4]);


%% Inicio de conexión con ROS
rosinit; %Conexión con nodo maestro

%% Cambio velocidad de los motores
baseVel =  rossvcclient('/base_controller/set_speed');                %Creación del cliente
baseVelMsg = rosmessage('dynamixel_controllers/SetSpeedRequest');     %Creación del mensaje
baseVelMsg.Speed = 0.5;                                               %Mensaje
call(baseVel,baseVelMsg);                                 %Envío

motor2Vel =  rossvcclient('/motor2_controller/set_speed');            %Creación del cliente
motor2VelMsg = rosmessage('dynamixel_controllers/SetSpeedRequest');   %Creación del mensaje
motor2VelMsg.Speed = 0.5;                                             %Mensaje
call(motor2Vel,motor2VelMsg);                             %Envío

motor3Vel =  rossvcclient('/motor3_controller/set_speed');            %Creación del cliente
motor3VelMsg = rosmessage('dynamixel_controllers/SetSpeedRequest');   %Creación del mensaje
motor3VelMsg.Speed = 0.5;                                             %Mensaje
call(motor3Vel,motor3VelMsg);                             %Envío

motor4Vel =  rossvcclient('/motor4_controller/set_speed');            %Creación del cliente
motor4VelMsg = rosmessage('dynamixel_controllers/SetSpeedRequest');   %Creación del mensaje
motor4VelMsg.Speed = 0.5;                                             %Mensaje
call(motor4Vel,motor4VelMsg);                             %Envío

gripperVel =  rossvcclient('/gripper_controller/set_speed');          %Creación del cliente
gripperVelMsg = rosmessage('dynamixel_controllers/SetSpeedRequest');  %Creación del mensaje
gripperVelMsg.Speed = 2;                                              %Mensaje
call(gripperVel,gripperVelMsg);                           %Envío
%% Cambio torque máximo de los motores
baseTorq =  rossvcclient('/base_controller/set_torque_limit');                    %Creación del cliente
baseTorqMsg = rosmessage('dynamixel_controllers/SetTorqueLimitRequest');          %Creación del mensaje
baseTorqMsg.TorqueLimit = 1;                                                      %Mensaje
call(baseTorq,baseTorqMsg);                                           %Envío

motor2Torq =  rossvcclient('/motor2_controller/set_torque_limit');                %Creación del cliente
motor2TorqMsg = rosmessage('dynamixel_controllers/SetTorqueLimitRequest');        %Creación del mensaje
motor2TorqMsg.TorqueLimit = 1;                                                    %Mensaje
call(motor2Torq,motor2TorqMsg);                                       %Envío

motor3Torq =  rossvcclient('/motor3_controller/set_torque_limit');                %Creación del cliente
motor3TorqMsg = rosmessage('dynamixel_controllers/SetTorqueLimitRequest');        %Creación del mensaje
motor3TorqMsg.TorqueLimit = 1;                                                    %Mensaje
call(motor3Torq,motor3TorqMsg);                                       %Envío

motor4Torq =  rossvcclient('/motor4_controller/set_torque_limit');                %Creación del cliente
motor4TorqMsg = rosmessage('dynamixel_controllers/SetTorqueLimitRequest');        %Creación del mensaje
motor4TorqMsg.TorqueLimit = 1;                                                    %Mensaje
call(motor4Torq,motor4TorqMsg);                                       %Envío

gripperTorq =  rossvcclient('/gripper_controller/set_torque_limit');              %Creación del cliente
gripperTorqMsg = rosmessage('dynamixel_controllers/SetTorqueLimitRequest');       %Creación del mensaje
gripperTorqMsg.TorqueLimit = 1;                                                   %Mensaje
call(gripperTorq,gripperTorqMsg);                                     %Envío

%% Cambio margen de error
baseMargin =  rossvcclient('/base_controller/set_compliance_margin');                    %Creación del cliente
baseMarginMsg = rosmessage('dynamixel_controllers/SetComplianceMarginRequest');          %Creación del mensaje
baseMarginMsg.Margin = 0;                                                      %Mensaje
call(baseMargin,baseMarginMsg);                                           %Envío

motor2Margin =  rossvcclient('/motor2_controller/set_compliance_margin');                %Creación del cliente
motor2MarginMsg = rosmessage('dynamixel_controllers/SetComplianceMarginRequest');        %Creación del mensaje
motor2MarginMsg.Margin = 0;                                                   %Mensaje
call(motor2Margin,motor2MarginMsg);                                       %Envío

motor3Margin =  rossvcclient('/motor3_controller/set_compliance_margin');                %Creación del cliente
motor3MarginMsg = rosmessage('dynamixel_controllers/SetComplianceMarginRequest');        %Creación del mensaje
motor3MarginMsg.Margin = 0;                                                   %Mensaje
call(motor3Margin,motor3MarginMsg);                                       %Envío

motor4Margin =  rossvcclient('/motor4_controller/set_compliance_margin');                %Creación del cliente
motor4MarginMsg = rosmessage('dynamixel_controllers/SetComplianceMarginRequest');        %Creación del mensaje
motor4MarginMsg.Margin = 0;                                                   %Mensaje
call(motor4Margin,motor4MarginMsg);                                       %Envío

gripperMargin =  rossvcclient('/gripper_controller/set_compliance_margin');              %Creación del cliente
gripperMarginMsg = rosmessage('dynamixel_controllers/SetComplianceMarginRequest');       %Creación del mensaje
gripperMarginMsg.Margin = 0;                                                   %Mensaje
call(gripperMargin,gripperMarginMsg);                                     %Envío

%% Cambio punch
basePunch =  rossvcclient('/base_controller/set_compliance_punch');                    %Creación del cliente
basePunchMsg = rosmessage('dynamixel_controllers/SetCompliancePunchRequest');          %Creación del mensaje
basePunchMsg.Punch = 60;                                                      %Mensaje
call(basePunch,basePunchMsg);                                           %Envío

motor2Punch =  rossvcclient('/motor2_controller/set_compliance_punch');                %Creación del cliente
motor2PunchMsg = rosmessage('dynamixel_controllers/SetCompliancePunchRequest');        %Creación del mensaje
motor2PunchMsg.Punch = 200;                                                   %Mensaje
call(motor2Punch,motor2PunchMsg);                                       %Envío

motor3Punch =  rossvcclient('/motor3_controller/set_compliance_punch');                %Creación del cliente
motor3PunchMsg = rosmessage('dynamixel_controllers/SetCompliancePunchRequest');        %Creación del mensaje
motor3PunchMsg.Punch = 60;                                                   %Mensaje
call(motor3Punch,motor3PunchMsg);

motor4Punch =  rossvcclient('/motor4_controller/set_compliance_punch');                %Creación del cliente
motor4PunchMsg = rosmessage('dynamixel_controllers/SetCompliancePunchRequest');        %Creación del mensaje
motor4PunchMsg.Punch = 60;                                                   %Mensaje
call(motor4Punch,motor4PunchMsg);  

gripperPunch =  rossvcclient('/gripper_controller/set_compliance_punch');              %Creación del cliente
gripperPunchMsg = rosmessage('dynamixel_controllers/SetCompliancePunchRequest');       %Creación del mensaje
gripperPunchMsg.Punch = 60;                                                   %Mensaje
call(gripperPunch,gripperPunchMsg);  %Envío
% 
%% Cambio Slope
baseSlope =  rossvcclient('/base_controller/set_compliance_slope');                    %Creación del cliente
baseSlopeMsg = rosmessage('dynamixel_controllers/SetComplianceSlopeRequest');          %Creación del mensaje
baseSlopeMsg.Slope = 200;                                                      %Mensaje
call(baseSlope,baseSlopeMsg);                                           %Envío

motor2Slope =  rossvcclient('/motor2_controller/set_compliance_slope');                %Creación del cliente
motor2SlopeMsg = rosmessage('dynamixel_controllers/SetComplianceSlopeRequest');        %Creación del mensaje
motor2SlopeMsg.Slope = 200;                                                   %Mensaje
call(motor2Slope,motor2SlopeMsg);                                       %Envío

motor3Slope =  rossvcclient('/motor3_controller/set_compliance_slope');                %Creación del cliente
motor3SlopeMsg = rosmessage('dynamixel_controllers/SetComplianceSlopeRequest');        %Creación del mensaje
motor3SlopeMsg.Slope = 200;                                                   %Mensaje
call(motor3Slope,motor3SlopeMsg);

motor4Slope =  rossvcclient('/motor4_controller/set_compliance_slope');                %Creación del cliente
motor4SlopeMsg = rosmessage('dynamixel_controllers/SetComplianceSlopeRequest');        %Creación del mensaje
motor4SlopeMsg.Slope = 200;                                                   %Mensaje
call(motor4Slope,motor4SlopeMsg);

gripperSlope =  rossvcclient('/gripper_controller/set_compliance_slope');              %Creación del cliente
gripperSlopeMsg = rosmessage('dynamixel_controllers/SetComplianceSlopeRequest');       %Creación del mensaje
gripperSlopeMsg.Slope= 200;                                                   %Mensaje
call(gripperSlope,gripperSlopeMsg);                                     %Envío

%% Publicar en el tópico de comando de posición del servomotor.

basePub = rospublisher('/base_controller/command','std_msgs/Float64'); % Creación publicador para la base
baseMsg = rosmessage(basePub); % Creación de mensaje para la base

motor2Pub = rospublisher('/motor2_controller/command','std_msgs/Float64'); % Creación publicador para el motor 2
motor2Msg = rosmessage(motor2Pub); % Creación de mensaje para el motor 2

motor3Pub = rospublisher('/motor3_controller/command','std_msgs/Float64'); % Creación publicador para el motor 3
motor3Msg = rosmessage(motor3Pub); % Creación de mensaje para el motor 3

motor4Pub = rospublisher('/motor4_controller/command','std_msgs/Float64'); % Creación publicador para el motor 4
motor4Msg = rosmessage(motor4Pub); % Creación de mensaje para el motor 4

gripperPub = rospublisher('/gripper_controller/command','std_msgs/Float64'); % Creación publicador para el gripper
gripperMsg = rosmessage(gripperPub); % Creación de mensaje para el gripper

%% Envíar mensaje HOME
baseMsg.Data = 2.618; % Valor del mensaje
send(basePub,baseMsg); % Envio

motor2Msg.Data = 1.0379; % Valor del mensaje
send(motor2Pub,motor2Msg); % Envio

motor3Msg.Data = 2.618; % Valor del mensaje
send(motor3Pub,motor3Msg); % Envio

motor4Msg.Data = 2.618; % Valor del mensaje
send(motor4Pub,motor4Msg); % Envio

gripperMsg.Data = 0; % Valor del mensaje
send(gripperPub,gripperMsg); % Envio


cla();
trplot(eye(4),'length',2,'rgb');
hold on
Phantom_X.plot([0 0 0 0],'workspace',[-40 50 -40 50 -40 50],'scale',0.3,'jaxes');
axis([-40 50 -40 50 -40 50])

q_alt = [0 0 0 0];
M = eye(4);
Mp = ones(1, Phantom_X.n);
for i=1:Phantom_X.n
    M = M * L(i).A(q_alt(i));
    Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',4,'text_opts', {'FontSize', 5});
end

%% Subscripción al tópico de estado del servomotor.

baseSub = rossubscriber('/base_controller/state','dynamixel_msgs/JointState'); % Subscriptor a estado de la base

motor2Sub = rossubscriber('/motor2_controller/state','dynamixel_msgs/JointState'); % Subscriptor a estado del motor 2

motor3Sub = rossubscriber('/motor3_controller/state','dynamixel_msgs/JointState'); % Subscriptor a estado del motor 3

motor4Sub = rossubscriber('/motor4_controller/state','dynamixel_msgs/JointState'); % Subscriptor a estado del motor 4

gripperSub = rossubscriber('/gripper_controller/state','dynamixel_msgs/JointState'); % Subscriptor a estado del gripper



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


counter1=1; 
counter2=1; 
proof=0;

while 1
     
    if length(List{1,counter1}) == 5 && proof == 0
        T = [List{1,counter1}(1),List{1,counter1}(2),List{1,counter1}(3),List{1,counter1}(4),List{1,counter1}(5)];
        counter1=counter1+1;
    else
        T = [List{1,counter1}{1,counter2}(1),List{1,counter1}{1,counter2}(2),List{1,counter1}{1,counter2}(3),List{1,counter1}{1,counter2}(4),List{1,counter1}{1,counter2}(5)];
        if counter2 == length(List{1,counter1})
            proof = 0;
            counter2=1;
            counter=counter+1;
        else
            counter2=counter2+1;
            proof = 1;
        end
        
        
%         for j=1:length(List{1,i})
%             
%             [Q1,Q2]=InvKin(List{1,i}{1,j}(1),List{1,i}{1,j}(2),List{1,i}{1,j}(3),List{1,i}{1,j}(4));
%             graficar(Phantom_X, L, Q2, 5)
%         end
    end
    
%     prompt = ('Vector de [x,y,z,phi,Gripper]:');
%     T = input(prompt);
    
    
    
    
    for i=1:Phantom_X.n
        delete(Mp(i))
    end
    
    %% Envíar mensaje
    
    T(4) = (T(4).*(pi/180));
    T(5) = (T(5).*(pi/180));
    
    
    [Q1,Q2]=InvKin(T(1),T(2),T(3),T(4))
    
    baseMsg.Data = Q2(1)+2.618; % Valor del mensaje
    send(basePub,baseMsg); % Envio

    motor2Msg.Data = Q2(2)+1.0379; % Valor del mensaje
    send(motor2Pub,motor2Msg); % Envio

    motor3Msg.Data = Q2(3)+2.618; % Valor del mensaje
    send(motor3Pub,motor3Msg); % Envio

    motor4Msg.Data = Q2(4)+2.618; % Valor del mensaje
    send(motor4Pub,motor4Msg); % Envio

    gripperMsg.Data = T(5); % Valor del mensaje
    send(gripperPub,gripperMsg); % Envio
    
    b1=0;
    b2=0;
        
    while 1
        %% Consulta de la posición
        
        base   = receive(baseSub); 
        motor2 = receive(motor2Sub); 
        motor3 = receive(motor3Sub); 
        motor4 = receive(motor4Sub); 
        gripper= receive(gripperSub); 
        
        baseAngle   = base.CurrentPos-2.618; % Angulo de la base
        motor2Angle = motor2.CurrentPos-1.0379; % Angulo del motor 2
        motor3Angle = motor3.CurrentPos-2.618; % Angulo del motor 3
        motor4Angle = motor4.CurrentPos-2.618; % Angulo del motor 4
        gripperAngle= gripper.CurrentPos; % Angulo del gripper
        
        %% Grafica
        trplot(eye(4),'length',2,'rgb');
        hold on
        Phantom_X.plot([baseAngle motor2Angle motor3Angle motor4Angle],'workspace',[-40 50 -40 50 -40 50],'scale',0.3,'jaxes');
        %
        % Workspace necesario para robots con juntas P
        % scale Robot refinado
        % jaxes Ejes de articulación
        axis([-40 50 -40 50 -40 50])
        
        b1 = abs(base.Error)+abs(motor2.Error)+abs(motor3.Error)+abs(motor4.Error)+abs(gripper.Error);
        
        if b1 == b2
            
            q_alt = [baseAngle motor2Angle motor3Angle motor4Angle];
            M = eye(4);
            Mp = ones(1, Phantom_X.n);
            for i=1:Phantom_X.n
                M = M * L(i).A(q_alt(i));
                Mp(i)=trplot(M,'rgb','frame',num2str(i),'length',4,'text_opts', {'FontSize', 5});
            end
            
            break
        end
        
        b2 = b1;
        
    end
end
%% Finalizar la conexión con el nodo maestro
rosshutdown; %Desconexión con nodo maestro



%%
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

%%