clear; clc; close all;

% Selecciona el archivo .ply
[file, path] = uigetfile('*.ply', 'Selecciona archivo PLY'); 
if isequal(file, 0)
    disp('No se seleccionó ningún archivo.');
    return;
end

filename = fullfile(path, file);
ptCloud = pcread(filename);
xyz = ptCloud.Location;


% Normalizar altura entre 0 y 1
z = xyz(:,3);
zMin = min(z);
zMax = max(z);
zNorm = (z - zMin) / (zMax - zMin);

% Escala de grises
colors = [zNorm, zNorm, zNorm];

% Representar nube 3D
figure;
pcshow(xyz, colors, 'MarkerSize', 30);
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Nube de puntos');
view(3);