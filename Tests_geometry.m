clear all 
clc

% clear all
% clc

% clear cam
% camlist = webcamlist
% cam = webcam(1);
% preview(cam); 
% 
% shot = snapshot(cam);
% 


img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatr�nica\Semestre 10\Rob�tica\Matlab\Figuras.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatr�nica\Semestre 10\Rob�tica\Matlab\moneda.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatr�nica\Semestre 10\Rob�tica\Matlab\cuadrado.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatr�nica\Semestre 10\Rob�tica\Matlab\CuadradoRot.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatr�nica\Semestre 10\Rob�tica\Matlab\circulo.jpg'); % Lee la imagen guardada

identifyGeometry(img);

