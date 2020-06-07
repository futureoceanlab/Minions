clear;

%% Load camera parameters
load("params.mat", 'params');
cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
    params.RotationOfCamera2, params.TranslationOfCamera2);
F = params.FundamentalMatrix;
f1 = params.CameraParameters1.FocalLength;
p1 = params.CameraParameters1.PrincipalPoint;
pxPitch = 0.0022;

intrinsics = [f1(1), 0, p1(1);
              0, f1(2), p1(2);
              0, 0, 1]; % mm
N = 8; % F-stop
fl = f1(1) * pxPitch;
desiredResolution = 0.020; %mm/px
WD = fl * desiredResolution/pxPitch;
rng(1,'twister');

imageSizeY = 1944;
imageSizeX = 2592;


dirPath = sprintf('bokeh_simulationImg_%d_%d', sizeIdx, vIdx);
if ~exist(dirPath, 'dir')
    mkdir(dirPath)
end
%% Generate and filter particles that do not fall in the image frame with buffer
% Create simulated particles in 3D
% origin is the center of the camera X-axis points right, Y-axis points bototm 
nParticles = 200;
centroids = generateCentroids(nParticles, -50, 50, -50, 50, 220, 280);

lPts = projectPoints(centroids, cameraMatrix1');
rPts = projectPoints(centroids, cameraMatrix2');

centroidsInFrame = [];
buffer = 100;
minX = -buffer; maxX = imageSizeX + buffer;
minY = -buffer; maxY = imageSizeY + buffer;
for i = 1:nParticles
    if inBoundary2D(lPts(:, i), minX, maxX, minY, maxY)... 
        || inBoundary2D(rPts(:, i), minX, maxX, minY, maxY)
        centroidsInFrame = [centroidsInFrame; centroids(i, :)];
    end
end
nParticlesInFrame = size(centroidsInFrame, 1);

%% Generate 3D point clouds given the shape (cylinder + sphere) per centroids

nCylinders = 0;
nSpheres = nParticlesInFrame - nCylinders;
nCylinderPts = 1e6;
nSpherePts = 1e2;

% All in mm
cHeightMin = 0.3; cHeightMax = 2;
cRadiusMin = 0.025; cRadiusMax = 1;
sDiameterMin = 0.05; sDiameterMax = 1;

cHeights = (cHeightMax - cHeightMin) * rand([nCylinders, 1]) + cHeightMin;
cRadii = (cRadiusMax - cRadiusMin) * rand([nCylinders, 1]) + cRadiusMin;
sRadius = (sDiameterMax - sDiameterMin) * rand([nSpheres, 1]) + sDiameterMin;
% sRadius = ones(nSpheres, 1).*sizeDistribution(sizeIdx);

% Cylinders
yawD = 0; 
pitchD = 90*rand([nCylinders, 1]) -45;
rollD = 90*rand([nCylinders, 1]) - 45;
cylinders = zeros(nCylinders, nCylinderPts, 3);
spheres = zeros(nSpheres, (nSpherePts+1)^2, 3);
cylinderVol = pi * (cRadii.^2).*cHeights;
sphereVol = 4/3 * pi * (sRadius).^3;
cylinderSpeed = cylinderVol/2;
sphereSpeed = sphereVol;
% Generate 3D cloud points of cylinders and spheres
for i = 1:nParticlesInFrame
    if i <= nCylinders
        cylinderPts = generateCylinder(cRadii(i), cHeights(i), nCylinderPts);
        cylinderPts = rotateCloud(cylinderPts, yawD, pitchD(i), rollD(i));
        cylinderPts = bsxfun(@plus, cylinderPts, centroidsInFrame(i, :));
        cylinders(i, :, :) = cylinderPts;
    else
        k = i - nCylinders;
        spherePts = generateSphere(nSpherePts, sRadius(k));     
        spherePts = bsxfun(@plus, spherePts, centroidsInFrame(i, :));
        spheres(i-nCylinders, :, :) = spherePts;
    end
end


%% Capture particles image and move frame by frame
[~, lCameraPos] = extrinsicsToCameraPose(eye(3),...
                                        zeros(1, 3));
[~, rCameraPos] = extrinsicsToCameraPose(params.RotationOfCamera2,...
                                        params.TranslationOfCamera2);
camPoses = [lCameraPos; rCameraPos];
nFrames = 20;
speed = 0.3;
imgPair = zeros(2, imageSizeY, imageSizeX) + 50;
matchedPoints1 = [];
matchedPoints2 = [];
[columnsInImage rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY);
camMatrix = zeros([2, size(cameraMatrix1)]);
camMatrix(1, :, :) = cameraMatrix1;
camMatrix(2, :, :) = cameraMatrix2;

id=1:nParticlesInFrame;
coord2D = zeros(2, nParticlesInFrame, 2);
coord3D = zeros(nParticlesInFrame, 3);
shapes = zeros(nParticlesInFrame, 1);

tracks = table();
for frameI=1:nFrames
    for i=1:nParticlesInFrame
        imgTemp = zeros(imageSizeY, imageSizeX);
        wasInFrame = [0 0];

        if frameI==1
            trace = particleTrace;
            newTrack = table(i, 0, 0, 0, 0, ...
                 255*rand(3, 1)', [NaN NaN], [NaN NaN NaN],...
                 [NaN NaN], {trace}, ...
                'VariableNames', {'id', 'age',  'totalVisibleCount', ...
                'totalInvisibleCount', 'consecutiveInvisibleCount', 'colour', ...
                'centroid', 'worldCoordinates', 'rightCoord2D', 'trace'
            });
            tracks = [tracks; newTrack];
        end
        for camI=1:2
            if (i <= nCylinders)
                curPts = squeeze(cylinders(i, :, :));
                shapes(i) = 1;
            else 
                k = i - nCylinders;
                curPts = squeeze(spheres(k, :, :));
            end
            pts = round(projectPoints(curPts, squeeze(camMatrix(camI, :, :))'))+1;
            coord2D(camI, i, :) = mean(pts, 2);
            coord3D(i, :) = mean(curPts)';
            dist = norm(centroidsInFrame(i, :) - camPoses(camI, :));
            COC = fl * fl / (N * (WD - fl))* (abs(dist - WD) / dist)/pxPitch/2;
            ptsMax = max(1, min([imageSizeX; imageSizeY], max(pts, [], 2) + floor(3*COC)));
            ptsMin = min([imageSizeX; imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

            for j = 1:size(pts, 2)
                x = pts(1, j);
                y = pts(2, j);
                if inBoundary2D(pts(:, j), 1, imageSizeX, 1, imageSizeY)
                    imgTemp(y, x) = 255;
                    wasInFrame(camI) = 1;
                end
            end
            imgTemp2 = imgaussfilt(imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), COC);
%             imgTemp2 = imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1));
            imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = imgTemp2;
            imgPair(camI, :, :) = squeeze(imgPair(camI, :, :)) + imgTemp;
        end
        if wasInFrame(1) == 1
            tracks.centroid(i, :) = squeeze(coord2D(1, i, :));
            if wasInFrame(2) == 1
                tracks.rightCoord2D(i, :) = squeeze(coord2D(2, i, :));
                tracks.worldCoordinates(i, :) = coord3D(i, :);
            end
            tracks.age(i) = tracks.age(i) + 1;
            tracks.totalVisibleCount(i) = tracks.totalVisibleCount(i) + 1;
            tracks.trace{i} = tracks.trace{i}.addDetected(squeeze(coord2D(1, i, :)));
        else
            if tracks.age(i) > 0
                tracks.age(i) = tracks.age(i) + 1;
                tracks.totalInvisibleCount(i) = tracks.totalInvisibleCount(i) + 1;
                tracks.consecutiveInvisibleCount(i) = tracks.consecutiveInvisibleCount(i) + 1;
            end
        end
    end
    img1Path = sprintf('%s/bokeh_cam_1_%d.tif', dirPath, frameI);
    img2Path = sprintf('%s/bokeh_cam_2_%d.tif', dirPath, frameI);
    imwrite(squeeze(imgPair(1, :, :))./255, img1Path, 'TIFF');
    imwrite(squeeze(imgPair(2, :, :))./255, img2Path, 'TIFF');
  
    imgPair = zeros(2, imageSizeY, imageSizeX) + 50;
    cylinders(:, :, 2) = cylinders(:, :, 2) + cylinderSpeed;
    spheres(:, :, 2) = spheres(:, :, 2) + sphereSpeed;
%     figure; 
%     imshow(squeeze(imgPair(1, :, :)), [0, 255]);
%     figure;
%     imshow(squeeze(imgPair(2, :, :)), [0, 255]);
end

% Remove tracks that did not appear at all
tracks(tracks.age == 0, :) = [];
trackFile = sprintf('%s/tracksSimulated.mat', dirPath);
save(trackFile, 'tracks');
