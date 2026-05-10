% Visualizar un archivo .ply en 2D
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
x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);

% Normalizar altura entre 0 y 1
zMin = min(z);
zMax = max(z);
zNorm = (z - zMin) / (zMax - zMin);

% Escala de grises
colors = repmat(zNorm, 1, 3);

% Representación 2D
figure;
scatter(x, y, 5, colors, 'filled');
axis equal;
grid on;
xlabel('X');
ylabel('Y');
title('Proyección 2D de nube de puntos');
view(2);