function [vol, maxVol] = computeOverlapVol(B, theta, focalLen, dof1, dof2, wantMaxVol)
    maxVol = 0;
    p1 = [1944 2592]./2;
    warning('off', 'MATLAB:alphaShape:DupPointsBasicWarnId');
    R1=  eye(3);
    T1 = [0 0 0];

%     B = 160;
%     deltas = deg2rad(-29:-0.2:-34);
    vol = zeros(size(theta));
%     focalLen = 33.52;
    pxPitch = 0.0022;
    desiredResolution = 0.02;

    for dIdx=1:size(theta, 1)
        delta = theta(dIdx);
        R2 = [cos(delta) 0 sin(delta); 0 1 0; -sin(delta) 0 cos(delta)]; %[1 0 0; 0 cos(delta) -sin(delta); 0 sin(delta) cos(delta)]; % params.RotationOfCamera2;
        T2 = -[B*cos(delta/2), 0, B*sin(delta/2)] ;

        [shpL, pts3DL] = computeShpPts(dof1, dof2, R1, T1, focalLen, p1, pxPitch, desiredResolution);
        [shpR, pts3DR] = computeShpPts(dof1, dof2, R2, T2, focalLen, p1, pxPitch, desiredResolution);
        x1 = pts3DL(:, 1);
        y1 = pts3DL(:, 2);
        z1 = pts3DL(:, 3);
        x2 = pts3DR(:, 1);
        y2 = pts3DR(:, 2);
        z2 = pts3DR(:, 3);
        id1 = inShape(shpR, x1, y1, z1);
        id2 = inShape(shpL, x2, y2, z2);
        shp3=alphaShape([x1(id1); x2(id2)], [y1(id1); y2(id2)], [z1(id1); z2(id2)], 3.5);
        vol(dIdx) = volume(shp3);
    end
    if (wantMaxVol)
        % TODO
        return;
    end
end