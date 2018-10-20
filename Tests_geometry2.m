%clear cam
clc
clear all
close all
% camlist = webcamlist
% cam = webcam(2);
% preview(cam); 

%shot = snapshot(cam);

%image(shot);

img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\prueba.jpg'); % Lee la imagen guardada

%[X,Y,isCircle] = identifyGeometry(shot)
[X,Y,isCircle] = identifyGeometry(img)
