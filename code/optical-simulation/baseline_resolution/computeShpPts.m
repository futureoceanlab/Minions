function [shp, pts3D] = computeShpPts(dof, R, T, focalLen, p1, pxPitch, desiredRes)
%     R = camMat(1:3, 1:3);
%     t = camMat(1:3, 4);
%     invCamMat  = [inv(R), -inv(R)*inv(t)];
%     invCamMat = [invCamMat; 0 0 0 1];
%     camMat = [camMat, ones(4, 1)];
%     invCamMat = inv(camMat);
%     f1 = [33.52/0.0022 33.52/0.0022]; %camParams.FocalLength;
    f1 = [focalLen/pxPitch, focalLen/pxPitch]; 
%     p1 = camParams.PrincipalPoint;

%     focalLen = 33.52; %f1 * pxPitch;
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
    for dIdx=1:size(d, 2)
        depth = d(dIdx);
        for pIdx=1:size(pts2D_H, 2)
            pts2D = (depth.*inv(intrinsics)*[pts2D_H(1, pIdx); pts2D_H(2, pIdx); 1]);
            pts2D = R*(pts2D - T');
            pts3D(dIdx, pIdx, :) = pts2D;
        end
    end
    pts3D = reshape(pts3D, size(pts3D, 1)*size(pts3D, 2), 3).*0.1; % Convert to cm
    
    pts3D2 = zeros(2, size(pts2D_S, 2), 3);
    for pIdx=1:size(pts2D_S, 2)
        pts2D = (minDepth.*inv(intrinsics)*[pts2D_S(1, pIdx); pts2D_S(2, pIdx); 1]);
        pts2D = R*(pts2D - T');
        pts3D2(1, pIdx, :) = pts2D;
        pts2D = (maxDepth.*inv(intrinsics)*[pts2D_S(1, pIdx); pts2D_S(2, pIdx); 1]);
        pts2D = R*(pts2D - T');
        pts3D2(2, pIdx, :) = pts2D;
    end
    pts3D2 = reshape(pts3D2, size(pts3D2, 1)*size(pts3D2, 2), 3).*0.1; % Convert to cm
    pts3D = [pts3D; pts3D2];
    shp = alphaShape(pts3D(:, 1), pts3D(:, 2), pts3D(:, 3), 3.5);
end



% for dIdx = d
%     minPts2D = zeros(size(pts2D_H));
%     maxPts2D = zeros(size(pts2D_H));
% end
% for pIdx=1:size(pts2D_H, 2)
%     minPts2D(:, pIdx) = (minDepth.*inv(intrinsics) * [pts2D_H(1, pIdx); pts2D_H(2, pIdx); 1]);
%     maxPts2D(:, pIdx) = (maxDepth.*inv(intrinsics) * [pts2D_H(1, pIdx); pts2D_H(2, pIdx); 1]);
% end
% pts3D = [minPts2D, maxPts2D]';
% points = projectPoints(pts3D, cameraMatrix2')';

% minY = max([0, min([points(1, 2), points(2, 2)])-10]);
% maxY = min([1944, max([points(1, 2), points(2, 2)])+10]);
% minX = max([0, min([points(1, 1), points(2, 1)])-10]);
% maxX = min([2592, max([points(1, 1), points(2, 1)])+10]);
%           
          
% alpha = atan((2592/2)/(33.437/0.0022));
% d = 80;
% WD = 273:333;
% beta = acos(d./WD);
% a = tan(alpha);
% b = tan(beta);
% A = a.*(1+b.^2).*sqrt(b.^2-a.^2)./(b.^2*(1+a.^2)^2);
% B = 1./sqrt(1-a.^2.*b.^2)
% C = a.*(1+b.^2)./sqrt((b.^2-a.^2).*(1-a.^2.*b.^2));
% V = 4./3.*(d.^3.*a.^2)./(1-(a.^2).*(b.^2)).*(A+B.*atan(C)