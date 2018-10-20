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


img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\Figuras.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\moneda.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\cuadrado.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\CuadradoRot.jpg'); % Lee la imagen guardada

identifyGeometry(img);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\circulo.jpg'); % Lee la imagen guardada

identifyGeometry(img);

