% EVALUATE BASELINE
% Evalaute how a change in baseline impacts the depth error,
% maximum disparity, overlapping volume and confusion of two particles
% in the correspondence search

%% Configuration
fl = 33.52; %mm
px = 0.0022; %mm
f_px = fl/px;
WD = 304.88; %228; %mm
B = 150:280; % mm
B = repmat(B, 3, 1);
b = B./2;
DoF = 30; %mm
res = 0.02; %um/px
% We take the pan angle, theta, to be such that the cameras' optical axis
% cross at the working distance
theta = acos(b/WD);
% We want to know how a small shift (i.e., 0.5deg) impacts the 
% parameters of interest
theta(1, :) = theta(1, :) + deg2rad(0.5);
theta(3, :) = theta(3, :) - deg2rad(0.5);

% Compute the maximum disparity
disp = 2*f_px.*tan(-atan((2./B).*(WD.*sin(theta)-DoF./sin(theta))) + theta);

% Compute the depth error
tm = - atan(-1/f_px);
tp = atan(1/f_px);
depthErr = 1000./2.*res.*(1./(sin(2*theta+tm)) + 1./(sin(2*theta-tp)));

figure;
hold on;
title('Max. Disparity and Depth Error vs Baseline', 'FontSize', 12);
yyaxis left;
d2 = plot(B(1, :), disp(2, :)', '-', 'LineWidth', 2);
d1 = plot(B(1, :), disp(1, :)', '-', 'HandleVisibility','off', 'LineWidth', 2);
d3 = plot(B(1, :), disp(3, :)', '-', 'HandleVisibility','off', 'LineWidth', 2);

% d2 = plot(B(1, :), disp(1, :)', 'r');

yyaxis right
e2 = plot(B(1, :), depthErr, '--', 'LineWidth', 2);
e1 = plot(B(1, :), depthErr, '--', 'HandleVisibility','off', 'LineWidth', 2);
e3 = plot(B(1, :), depthErr, '--', 'HandleVisibility','off', 'LineWidth', 2);

yyaxis left
ylabel('Max Disparity (px)')
yyaxis right
ylabel(['Depth Error, ',char(181),'m/px'],'Interpreter','tex', 'FontSize', 12)
xlabel('Baseline (mm)', 'FontSize', 12);
legend(["Disparity", "Depth Error"], 'FontSize', 12);
ax = gca;
ax.XAxis.FontSize = 12;
ax.YAxis(1).FontSize = 12;
ax.YAxis(2).FontSize = 12;
%% Volume
grid on;

%v = zeros(size(theta));
% Overlapping volume was already computed based on the above range of 
% values of baseline and theta.
load('overlappingVol.mat', 'v');
% for bIdx = 11:size(B, 2)
%     bb = B(1, bIdx);
%     th = -(pi/2-theta(:, bIdx))*2;
% %     computeOverlapVol(bb, th, fl, 30, 0);
%     [v(:, bIdx), ~] = computeOverlapVol(bb, th, fl, 30, 0);
% end

%% Pan Angle
figure;
hold on;
title('Pan Angle VS Baseline');
ylabel('Pan Angle (deg)');
xlabel('Baseline (mm)');
plot(B(1, :), rad2deg(theta(2, :)));


%% Confusion probabiliity
sRadiusRange = [25 120; 120 195; 195 320; 320 520; 520 850; 850 1400]; % minRadius to maxRadius (um)
concentration = [300; 40; 18; 13; 1.5; 1.25];
curConcentration = (2 * concentration).*mean(sRadiusRange, 2);
imgVol = 2592*1944*(0.02^2)*60;
totalVol = 1e9; % mm^3
N = ceil(sum(curConcentration)*imgVol/totalVol)-1;
n = 1;
V = 2592*1944;
v = disp*20;
p = nchoosek(N, n)*((v/V).^n).*((1-v/V).^(N-n))*100;
color1 = [0, 0.4470, 0.7410];
figure; 
hold on;
title('Probability of Particle Confusion in Correspondence Search', 'FontSize', 12);
ylabel('Probability (%)');
xlabel('Baseline (mm)');
ax = gca;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
plot(B(1, :), p(2, :), '-', 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410]);
plot(B(1, :), p(1:2:3, :), '-', 'HandleVisibility','off', 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410]);
grid on;
hold off;