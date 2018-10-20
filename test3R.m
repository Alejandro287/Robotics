% Robot 3R

clear all 

l1 = 2;
l2 = 2;
l3 = 1;

l1 = 10.67;
l2 = 10.67;
l3 = 7.249;

L(1) = Link('revolute','alpha',0,'a',l1,'d',0,'qlim',[-3*pi/4 3*pi/4 ],'offset',0);
L(2) = Link('revolute','alpha',0,'a',l2,'d',0,'qlim',[-3*pi/4  3*pi/4 ],'offset',0);
L(3) = Link('revolute','alpha',0,'a',l3,'d',0,'qlim',[-3*pi/4  3*pi/4 ],'offset',0);
RRR = SerialLink(L,'name','3R');
RRR.plot([0 0 0],'top')
%%
x = 3;
y = 3;
phi = pi/2;

x = 10;
y = 10;
phi = 0;
%%
x0 = x - l3*cos(phi)
y0 = y - l3*sin(phi)
codo = 0;
if codo
    [qinv, check] = inv_kin_2R(x0,y0,l1,l2,1)
    q3 = phi - qinv(1) - qinv(2)
    q = [qinv; q3]'
else
    [qinv, check] = inv_kin_2R(x0,y0,l1,l2,0)
    q3 = phi - qinv(1) - qinv(2)
    q = [qinv; q3]'
end
%%
T = RRR.fkine(q);
RRR.plot(q,'top')
RRR.teach();
text(x+0.2,y,strcat('[',num2str(T(1,4)),',',num2str(T(2,4)),']'))