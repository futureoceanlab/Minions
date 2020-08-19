function [shp, pts3D] = computeShpPts(dof, R, T, focalLen, p1, pxPitch, desiredRes)
    % COMPUTE SHP PTS
    % Find the cloud points that lay on the surface of the imaging volume
    % projected from the imaging sensor
    
    f1 = [focalLen/pxPitch, focalLen/pxPitch]; 
    intrinsics = [f1(1), 0, p1(1);
                0, f1(2), p1(2);
                0, 0, 1]; % mm,
    WD = focalLen* desiredRes/pxPitch;
    minDepth = WD - dof;
    maxDepth = WD + dof;

    w = [1:72:2592, 2592];
    h = [73:72:1944, 1944];
    d = minDepth:5:maxDepth;
    % pts2D_H = zeros(3, (size(w, 2)+size(h, 2))*2-4);

    pts2D_H = [];
    % We pick pixels at the exterior of the imaging frame
    for wIdx = w
        pts2D_H = [pts2D_H, [wIdx; 1; 1]];
        pts2D_H = [pts2D_H, [wIdx; 1944; 1]];
    end
    for hIdx = h
        pts2D_H = [pts2D_H, [1; hIdx; 1]];
        pts2D_H = [pts2D_H, [2592; hIdx; 1]];
    end
    pts2D_S = [];
    for wIdx=w
        for hIdx=h
            pts2D_S = [pts2D_S, [wIdx; hIdx; 1]];
        end
    end

    pts3D = zeros(size(d, 2), size(pts2D_H, 2), 3);
    % Project the pixel points above to different depths
    for dIdx=1:size(d, 2)
        depth = d(dIdx);
        for pIdx=1:size(pts2D_H, 2)
            pts2D = (depth.*inv(intrinsics)*[pts2D_H(1, pIdx); pts2D_H(2, pIdx); 1]);
            pts2D = R*(pts2D - T');
            pts3D(dIdx, pIdx, :) = pts2D;
        end
    end
    % Convert to cm
    pts3D = reshape(pts3D, size(pts3D, 1)*size(pts3D, 2), 3).*0.1; 
    
    % We also need to project cloud points on the x-y plane at minimum
    % and maximum depths
    pts3D2 = zeros(2, size(pts2D_S, 2), 3);
    for pIdx=1:size(pts2D_S, 2)
        pts2D = (minDepth.*inv(intrinsics)*[pts2D_S(1, pIdx); pts2D_S(2, pIdx); 1]);
        pts2D = R*(pts2D - T');
        pts3D2(1, pIdx, :) = pts2D;
        pts2D = (maxDepth.*inv(intrinsics)*[pts2D_S(1, pIdx); pts2D_S(2, pIdx); 1]);
        pts2D = R*(pts2D - T');
        pts3D2(2, pIdx, :) = pts2D;
    end
    % Convert to cm
    pts3D2 = reshape(pts3D2, size(pts3D2, 1)*size(pts3D2, 2), 3).*0.1;
    pts3D = [pts3D; pts3D2];
    shp = alphaShape(pts3D(:, 1), pts3D(:, 2), pts3D(:, 3), 3.5);
end