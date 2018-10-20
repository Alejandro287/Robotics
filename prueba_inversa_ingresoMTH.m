clear all 
clc

pos = [20 10 30]';
phi = pi/4;

T = eye(4,4);
T = eul2tr([atan2(T(2,4),T(1,4)), phi, 0]);
T(1:3,4) = pos;
 
InvKin(pos(1), pos(2), pos(3), phi)
InvKin2(T)




function [Q1,Q2] = InvKin2(T)
    l1 = 14.04;
    l2 = 10.67;
    l3 = 10.67;
    l4 = 10.53;
    
    xff=T(1,4);
    yff=T(2,4);
    zff=T(3,4);
    eul=tr2eul(T)
    phi=eul(1,2);
    
    zf=zff-l1;
    q1=atan2(yff,xff);
    xf=sqrt((yff^2)+(xff^2));
        
    x0=xf-l4*cos(phi);
    z0=zf-l4*sin(phi);
    D=(x0^2+z0^2-l2^2-l3^2)/(2*l2*l3);
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

function [Q1,Q2] = InvKin(xff,yff,zff,phi)
    l1 = 14.04;
    l2 = 10.67;
    l3 = 10.67;
    l4 = 10.53;

      
    zf=zff-l1;
    q1=atan2(yff,xff);
    xf=sqrt((yff^2)+(xff^2));
        
    x0=xf-l4*cos(phi);
    z0=zf-l4*sin(phi);
    D=(x0^2+z0^2-l2^2-l3^2)/(2*l2*l3);
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