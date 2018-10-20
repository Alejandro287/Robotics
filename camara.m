clear cam
camlist = webcamlist
cam = webcam(1);
preview(cam); 

shot = snapshot(cam);

image(shot);
%%
 
clear all
clc
shot = imread('C:\Users\Alejandro\OneDrive - Universidad Nacional de Colombia\Documents\Universidad - Mecatrónica\Semestre 10\Robótica\Matlab\Figuras.jpg'); % Lee la imagen guardada
[f,c]=size(shot);
% figure
% imshow(shot)


shot = rgb2gray(shot);

% figure
% imshow(shot)

% shot1 = im2bw(shot, 0.9);
% 
% figure
% imshow(shot1)

% level = graythresh(shot)
% BW = imbinarize(shot,level);
% 
% figure
% imshow(BW)

%shot = medfilt2(shot,[round(f/200),round(c/200)]);

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

properties = regionprops(shot,'Area', 'BoundingBox', 'centroid','Perimeter','Extent','EquivDiameter')

[f,c]=size(shot);
area=0;
for i=2:f-1
    for j=2:c-1
        if shot(i,j)==1
            area = area + 1; 
        end
    end
end 


% [B,L] = bwboundaries(shot);
% 
% for k = 1:length(B)
%    boundary = B{k};
%    minboundrect(boundary(1), boundary(2))
%    %minBoundingBox(boundary)
%    %plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
% end


B = bwboundaries(shot,'noholes');


boundary = bwperim(shot); % Da contornos
% figure
% imshow(boundary);

figure
imshow(shot)
hold on
%[f,c]=size(boundary);

A=[];
counter=1;
for i=2:f-1
    for j=2:c-1
        if boundary(i,j)==1
            A(1,counter)=i;
            A(2,counter)=j;
            counter = counter + 1; 
        end
    end
end 

size(A);

%[rx,ry,areaCuad] = minboundrect(A(2,:), A(1,:));

[BLen_f,BLen_c]=size(B);
for i=1:BLen_f
    [rx,ry,areaCuad] = minboundrect(B{i}(:,2), B{i}(:,1));
    [center,radius]= minboundcircle(B{i}(:,2), B{i}(:,1));
    viscircles(center,radius);
    line(rx,ry);
    areaCir = round(pi*(radius^2));
    if abs(areaCir-properties(i).Area)<abs(areaCuad-properties(1).Area)
        disp('circulo')
    else
        disp('cuadrado')
    end
end



%pos = minBoundingBox(A);

%[center,radius]=minboundcircle(A(2,:), A(1,:));



%%

dst = cv.adaptiveThreshold(shot);
cnt = cv.findContours(dts);
rct = cv.minAreaRect(cnt);
box = cv.boxPoints(rct);
box = np.int0(box);
im = cv.drawContours(img,box)
[center,radius] = cv.minEnclosingCircle(cnt)
%(x,y),radius = cv.minEnclosingCircle(cnt)
x=center(1)
y=center(2)
center = [round(x),round(y)]
radius = round(radius)
img = cv.circle(img, center, radius)
%cv.circle(img,center,radius,(0,255,0),2)
figure
imshow(boundary);

%%

bw = im2bw(img,.36);


[centers, radii] = imfindcircles(bw,[30 65],'ObjectPolarity','dark');

    h = viscircles(centers,radii);
    
clear cam