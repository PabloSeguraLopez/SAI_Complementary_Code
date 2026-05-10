clear; clc; close all;

%% Cargar imagen
img = imread('mapa.png');

if size(img,3) == 3
    gray = rgb2gray(img);
else
    gray = img;
end

% Blanco = obstáculo, negro = libre
obstacles = gray > 128;

%% Inflar obstáculos por hitbox
hitboxSize = 8;
inflateRadius = ceil(hitboxSize / 2);

se = strel('square', 2*inflateRadius + 1);
obstaclesInflated = imdilate(obstacles, se);

freeMap = ~obstaclesInflated;

%% Seleccionar puntos
figure;
imshow(freeMap);
title('Clica primero el inicio y luego la meta sobre zona libre');

[x, y] = ginput(2);

start = round([y(1), x(1)]);  % [fila, columna]
goal  = round([y(2), x(2)]);

if ~freeMap(start(1), start(2))
    error('El punto inicial está en obstáculo o demasiado cerca de uno.');
end

if ~freeMap(goal(1), goal(2))
    error('El punto final está en obstáculo o demasiado cerca de uno.');
end

%% Ejecutar A*
path = astarGrid(freeMap, start, goal);

%% Mostrar resultado
figure;
imshow(gray);
hold on;

plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

if ~isempty(path)
    plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2);
    title('Ruta encontrada con A*');
else
    title('No se encontró ruta');
end

hold off;

%% Función A*
function path = astarGrid(freeMap, start, goal)

    [rows, cols] = size(freeMap);

    gScore = inf(rows, cols); % coste real acumulado del inicio a la celda
    fScore = inf(rows, cols); % coste heuristico

    gScore(start(1), start(2)) = 0;
    fScore(start(1), start(2)) = heuristic(start, goal);

    cameFrom = zeros(rows, cols, 2);

    openSet = false(rows, cols);
    openSet(start(1), start(2)) = true;

    closedSet = false(rows, cols);

    % Movimiento en 8 direcciones
    neighbors = [
        -1  0;
         1  0;
         0 -1;
         0  1;
        -1 -1;
        -1  1;
         1 -1;
         1  1
    ];

    while any(openSet(:))

        % Nodo con menor fScore
        tempF = fScore;
        tempF(~openSet) = inf;
        [~, idx] = min(tempF(:));
        [currentRow, currentCol] = ind2sub(size(freeMap), idx);

        current = [currentRow, currentCol];

        if isequal(current, goal)
            path = reconstructPath(cameFrom, current);
            return;
        end

        openSet(currentRow, currentCol) = false;
        closedSet(currentRow, currentCol) = true;

        for k = 1:size(neighbors,1)

            nr = currentRow + neighbors(k,1);
            nc = currentCol + neighbors(k,2);

            if nr < 1 || nr > rows || nc < 1 || nc > cols
                continue;
            end

            if ~freeMap(nr, nc) || closedSet(nr, nc)
                continue;
            end

            % Coste: 1 en horizontal/vertical, sqrt(2) en diagonal
            stepCost = norm(neighbors(k,:));
            tentativeG = gScore(currentRow, currentCol) + stepCost;

            if ~openSet(nr, nc)
                openSet(nr, nc) = true;
            elseif tentativeG >= gScore(nr, nc)
                continue;
            end

            cameFrom(nr, nc, :) = current;
            gScore(nr, nc) = tentativeG;
            fScore(nr, nc) = tentativeG + heuristic([nr, nc], goal);
        end
    end

    path = [];
end

%% Heurística euclídea
function h = heuristic(p1, p2)
    h = norm(double(p1) - double(p2));
end

%% Reconstrucción del camino
function path = reconstructPath(cameFrom, current)

    path = current;

    while true
        prev = squeeze(cameFrom(current(1), current(2), :))';

        if all(prev == 0)
            break;
        end

        path = [prev; path];
        current = prev;
    end
end