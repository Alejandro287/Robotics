
clc
clear all
clear L

L(1) = Revolute('d', 0.352, 'a', 0, 'alpha', 0, 'offset', 0, 'modified');
L(2) = Revolute('d', 0, 'a', 0.070, 'alpha', -pi/2, 'offset', -pi/2, 'modified');
L(3) = Revolute('d', 0, 'a', 0.360, 'alpha', 0, 'offset', 0, 'modified');
L(4) = Revolute('d', 0.380, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
L(5) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
L(6) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');


IRBBIN = SerialLink(L, 'name', 'IRB 140');

IRBBIN.tool = trotx(0)*transl([0,0,0])*trotz(0)*transl([0,0,0.065]);


graficar (IRBBIN, L, [0 0 0 0 0 0], 0)
 
 
Ori=[ 0.4918,   0.5240,  -0.69953,  0;
      0.7975,   0.0493,   0.6012,   0;
      0.3493,  -0.8503,  -0.3937,   0;
         0,        0,        0,     1];

%Pos = [0.0316 1.1862 0.5255];
Pos = [0.0316 0.81845, 0.5255];

Ori(1:3,4) = Pos;

MTH = Ori

Ikin = IRBBIN.ikunc(MTH)
Ikin*180/pi
%graficar (IRBBIN, L, Ikin, 0)
MTH2=IRBBIN.fkine(Ikin)
round(MTH2,3)==round(MTH,3)

function graficar (Robot, L, Q, escala)
    
    cla();

    tam = (sum(Robot.d)+sum(Robot.a))+escala;

    trplot(eye(4),'length',2,'rgb');
    hold on
    axis([-tam tam -tam tam -tam tam])
    Robot.plot(Q,'workspace',[-tam tam -tam tam -tam tam],'scale',0.3);
    Robot.teach()
    

end
