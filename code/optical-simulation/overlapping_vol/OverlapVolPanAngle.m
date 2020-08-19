% ignore alphashape warning
warning('off', 'MATLAB:alphaShape:DupPointsBasicWarnId');
% Translational and rotational matrix of the left camera
R1=  eye(3);
T1 = [0 0 0];
B = 160; % baseline
% Pan Angle to sweep
deltas = deg2rad(-27:-0.2:-33.2);
v = zeros(size(deltas)); % Volume
p1 = [1944, 2592]./2; % principal point
focalLen = 33.52;
for dIdx=1:size(deltas, 2)
    delta = deltas(dIdx);
    % translational and rotational matrix for the right camera
    R2 = [cos(delta) 0 sin(delta); 0 1 0; -sin(delta) 0 cos(delta)];
    T2 = -[B*cos(delta/2), 0, B*sin(delta/2)] ;
    % point clouds on the surface of the imaging volume of each camera
    [shpL, pts3DL] = computeShpPts(-30, 30, R1, T1, focalLen, p1, pxPitch, desiredResolution);
    [shpR, pts3DR] = computeShpPts(-30, 30, R2, T2, focalLen, p1, pxPitch, desiredResolution);
    % Find the overlapping volume 
    x1 = pts3DL(:, 1);
    y1 = pts3DL(:, 2);
    z1 = pts3DL(:, 3);
    x2 = pts3DR(:, 1);
    y2 = pts3DR(:, 2);
    z2 = pts3DR(:, 3);
    id1 = inShape(shpR, x1, y1, z1);
    id2 = inShape(shpL, x2, y2, z2);
    shp3=alphaShape([x1(id1); x2(id2)], [y1(id1); y2(id2)], [z1(id1); z2(id2)], 3.5);
    v(dIdx) = volume(shp3);
end
% Plot result
panAngle = 90+rad2deg(deltas)./2;

figure; hold on; 
title('Overlapping Volume vs Pan Angle', 'FontSize', 12); 
plot(panAngle, v, 'LineWidth', 2);
xlabel('Pan Angle (deg)');
ylabel('Overlapping Volume (mL)');
xlim([min(panAngle), max(panAngle)]);
ax = gca;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
grid on;
hold off;