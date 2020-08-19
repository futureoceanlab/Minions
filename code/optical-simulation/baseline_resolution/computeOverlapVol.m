function [vol, maxVol] = computeOverlapVol(B, thetas, focalLen, dof, wantMaxVol)
    % COMPUTE OVERLAP VOL
    % compute the overlapping imaging volume between two cameras given
    % the baseline, pan angle, focal length and depth of field
    
    maxVol = 0;
    % principal points
    p1 = [1944 2592]./2;
    % ignore alphashape warning
    warning('off', 'MATLAB:alphaShape:DupPointsBasicWarnId');
    % Translational and rotational matrix of the left camera
    R1=  eye(3);
    T1 = [0 0 0];

    vol = zeros(size(thetas));
    pxPitch = 0.0022;
    desiredResolution = 0.02;

    for tIdx=1:size(thetas, 1)
        theta = thetas(tIdx);
        % translational and rotational matrix for the right camera
        R2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        T2 = -[B*cos(theta/2), 0, B*sin(theta/2)] ;
        % point clouds on the surface of the imaging volume of each camera
        [shpL, pts3DL] = computeShpPts(dof, R1, T1, focalLen, p1,...
            pxPitch, desiredResolution);
        [shpR, pts3DR] = computeShpPts(dof, R2, T2, focalLen, p1,...
            pxPitch, desiredResolution);
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
        vol(tIdx) = volume(shp3);
    end
    if (wantMaxVol)
        % TODO
        % Sweep through the overlapping volumes and identify the setup
        % that outputs the maximum volume
        return;
    end
end