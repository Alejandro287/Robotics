
clc
clear all
clear L

L(1) = Revolute('d', 0.352, 'a', 0, 'alpha', 0, 'offset', 0, 'modified');
L(2) = Revolute('d', 0, 'a', 0.070, 'alpha', pi/2, 'offset', 0, 'modified');
L(3) = Revolute('d', 0, 'a', 0.360, 'alpha', 0, 'offset', 0, 'modified');
L(4) = Revolute('d', 0.380, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
L(6) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');

irb = SerialLink(L, 'name', 'IRB 140', ...
    'manufacturer', 'ABB', 'comment', 'modified DH');

traslacion = [0.0316, 1186.2, 525.5];
rotQuat = [0.53558, -0.67754, -0.48764, 0.12767];
eulZYX = quat2eul(rotQuat)