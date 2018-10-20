clear cam
camlist = webcamlist % Muestra las webcams
cam = webcam(1); % Selecciona la webcam
preview(cam); % Muestra que ve la webcam elegida
%%
img = snapshot(cam); % Toma un imagen como un arreglo de datos
im = image(img); % Convierte el arreglo en otro formato
saveas(im,'Prueba','png'); % Guarda la imagen como png
%%
shot = imread('Prueba.png'); % Lee la imagen guardada
figure
imshow(shot)

%%
p=shot(60:550,125:780);
size(p);
figure
imshow(p)
%%
bn = im2bw(p); % Vuelve binaria la imagen
figure
imshow(bn); 
boundary = bwperim(bn); % Da contornos
figure
imshow(boundary);

[L Ne] = bwlabel(bn); % Cuenta elementos y los etiqueta
figure
imshow(label2rgb(L));

prop = regionprops(L);

for n=1:length(prop)
        rectangle('Position',prop(n).BoundingBox,'EdgeColor','g','LineWidth',2)
end
