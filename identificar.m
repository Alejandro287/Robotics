
function isCircle = identify(shot)
    [f,c]=size(shot);
    % figure
    % imshow(shot)

    shot = rgb2gray(shot);

    % figure
    % imshow(shot)

    shot = adaptivethreshold(shot,[round(f/2),round(c/2)],0);
    % figure
    % imshow(shot)

    shot = medfilt2(shot,[round(f/200),round(c/200)]);
    % figure
    % imshow(shot)

    shot = ~shot;
    % figure
    % imshow(shot)
    shot = bwareaopen(shot, round((f*c)/100));
    % figure
    % imshow(shot)

    shot = imfill(shot,'holes');
    % figure
    % imshow(shot)

    properties = regionprops(shot,'Area', 'BoundingBox', 'centroid','Perimeter','Extent','EquivDiameter');

    B = bwboundaries(shot,'noholes');

    boundary = bwperim(shot); % Da contornos
    % figure
    % imshow(boundary);

    figure
    imshow(shot)
    hold on

    [BLen_f,BLen_c]=size(B);

    for i=1:BLen_f
        [rx,ry,areaCuad] = minboundrect(B{i}(:,2), B{i}(:,1));
        [center,radius]= minboundcircle(B{i}(:,2), B{i}(:,1));
        viscircles(center,radius);
        line(rx,ry);
        areaCir = round(pi*(radius^2));
        if abs(areaCir-properties(i).Area)<abs(areaCuad-properties(1).Area)
            disp('circulo')
            proof=1;
        else
            disp('cuadrado')
            proof=0;
        end
    end
    isCircle=proof;
    hold off
end


% clear all
% clc

% clear cam
% camlist = webcamlist
% cam = webcam(1);
% preview(cam); 
% 
% shot = snapshot(cam);
% 
% image(shot);
 
% img = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\Figuras.jpg'); % Lee la imagen guardada
% 
% ident(img)

