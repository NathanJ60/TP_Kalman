% Script de visualisation des données brutes - Exercice 1
clear all;
load('kf_linear_data_2.mat');

% Affichage des informations sur les données
disp('=== Informations sur les données ===');
disp(['Pas de temps (dt): ', num2str(dt), ' s']);
disp(['Nombre de pas de temps (T): ', num2str(size(groundtruth, 2))]);
disp('Variables d''état:');
disp(labels);
disp(' ');

% Figure 1: Trajectoire 2D
figure(1);
hold on;
plot(groundtruth(1,:), groundtruth(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(obspos(1,:), obspos(2,:), 'r.', 'MarkerSize', 8, 'DisplayName', 'Observations bruitées');
xlabel('x (m)');
ylabel('y (m)');
legend('Location', 'best');
axis equal;
grid on;
title('Trajectoire du robot - Position');
hold off;

% Figure 2: Position en fonction du temps
figure(2);
T = size(groundtruth, 2);
temps = (0:T-1) * dt;

subplot(2,1,1);
hold on;
plot(temps, groundtruth(1,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(temps, obspos(1,:), 'r.', 'MarkerSize', 6, 'DisplayName', 'Observations');
xlabel('Temps (s)');
ylabel('x (m)');
legend('Location', 'best');
grid on;
title('Position X au cours du temps');
hold off;

subplot(2,1,2);
hold on;
plot(temps, groundtruth(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Vérité terrain');
plot(temps, obspos(2,:), 'r.', 'MarkerSize', 6, 'DisplayName', 'Observations');
xlabel('Temps (s)');
ylabel('y (m)');
legend('Location', 'best');
grid on;
title('Position Y au cours du temps');
hold off;

% Figure 3: Vitesses réelles
figure(3);
subplot(2,1,1);
plot(temps, groundtruth(3,:), 'b-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('v_x (m/s)');
grid on;
title('Vitesse selon X (vérité terrain)');

subplot(2,1,2);
plot(temps, groundtruth(4,:), 'g-', 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('v_y (m/s)');
grid on;
title('Vitesse selon Y (vérité terrain)');

disp('Visualisation terminée !');
