clear all;

% Load 'dt', 'groundtruth', 'labels', 'obspos'
load linear.mat;
N = size(groundtruth, 1);
T = size(groundtruth, 2);
livedisplay = false;

% TODO: Initial filter state
X = ...; % (N*1)
P = ...; % (N*N)

% TODO: Evolution model and covariance of its noise
A = ...; % (N*N)
Q = ...; % (N*N)

% TODO: Observation model and covariance of its noise
C = ...; % (M*N)
R = ...; % (M*M)

% Memory for display after filtering
Xs = zeros(N, T);
Ps = zeros(N, N, T);
I = eye(N);

for t = 1:T
    % TODO: Kalman Update
    y = obspos(:, t);
    K = ...;
    X = ...;
    P = ...;

    % Storage
    Xs(:, t) = X;
    Ps(:, :, t) = P;

    % TODO: Prediction
    X = ...;
    P = ...;

    % Live display
    if livedisplay
        figure(3);
        clf;
        hold on;
        plot(groundtruth(1, 1:T), groundtruth(2, 1:T), 'k', 'DisplayName', 'Groundtruth', Xs(1, 1:t), Xs(2, 1:t), 'r' , 'DisplayName', 'History', X(1), X(2), 1, 'b', 'DisplayName', 'Estimate');
        displayCov(X(1:2), P(1:2,1:2), 0.997, 'b');
        axis equal;
        legend;
        xlabel(num2str(t));
        hold off;
        pause(0.001);
    end
end

figure(1);
title('Trajectory and estimate')
hold on;
plot(Xs(1, 1:T), Xs(2, 1:T), 'b', 'DisplayName', 'Estimate');
plot(groundtruth(1, 1:T), groundtruth(2, 1:T), 'k', 'DisplayName', 'Groundtruth');
legend;
axis equal;
hold off;

% Error display with +/- estimated standard deviations
errors = Xs(:, 1:T) - groundtruth(:, 1:T);
stddevs = squeeze([sqrt(Ps(1, 1, 1:T)); sqrt(Ps(2, 2, 1:T)); sqrt(Ps(3, 3, 1:T)); sqrt(Ps(4, 4, 1:T))]);

figure(2);
title('Estimation errors and 3 std bounds');
for k = 1:N,
    subplot(N, 1, k);
    hold on;
    plot(1:T, errors(k, :), 'b');
    plot(1:T, 3 * stddevs(k, :), 'r');
    plot(1:T, -3 * stddevs(k, :), 'r');
    zoom on;
    ylabel(labels(k));
    hold off;
end

mean_error = mean(errors');
rmse = sqrt(mean(errors'.^2));
consistency = mean(abs(errors') < (3*stddevs'));
uncertainty = mean(stddevs');

disp(['Mean error: ', num2str(mean_error)]);
disp(['Consistency: ', num2str(consistency)]);
disp(['RMSE: ', num2str(rmse)]);
disp(['Uncertainty: ', num2str(uncertainty)]);