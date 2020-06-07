sizeBins = [73 120; 120 195; 195 320; 320 520; 520 850; 850 1400;]; %um

% lmg0901-ps1-250
sinkingRate = [90.1; 55.0; 56.4; 54.2; 157.6; 137.2; 134.7]; % m/day
concentration = [100; 30; 13; 8; 1; 0.8; 0.15]; % No./m^3
% %LMG0901-PS2-300
% sinkingRate = [254.0; 109.5; 85.7; 58.7; 60.6; 85.0; ];
% concentration = [200; 180; 100; 50; 1; 0.8; 0.15];


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


%% Generate and filter particles that do not fall in the image frame with buffer
% Create simulated particles in 3D
% origin is the center of the camera X-axis points right, Y-axis points bototm 
nParticles = round(sum(concentration));
Zmax = 500; 280;
Zmin = -500; 220;
d = Zmax - Zmin; % mm
boxLen = 1000; %round(sqrt(1e9 / d));
centroids = generateCentroids(nParticles, -boxLen/2, boxLen/2, -boxLen/2, boxLen/2, Zmin, Zmax);

lPts = projectPoints(centroids, cameraMatrix1');
rPts = projectPoints(centroids, cameraMatrix2');

centroidsInFrame = [];
buffer = 500;
minX = -buffer; maxX = imageSizeX + buffer;
minY = -buffer; maxY = imageSizeY + buffer;
for i = 1:nParticles
    if isInImage(lPts, i, minX, maxX, minY, maxY)... 
        || isInImage(rPts, i, minX, maxX, minY, maxY)
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
% sDiameters = (sDiameterMax - sDiameterMin) * rand([nSpheres, 1]) + sDiameterMin;
sRadius = ones(nSpheres, 1).*sizeDistribution(sizeIdx);

% Cylinders
yawD = 0; 
pitchD = 90*rand([nCylinders, 1]) -45;
rollD = 90*rand([nCylinders, 1]) - 45;
cylinders = zeros(nCylinders, nCylinderPts, 3);
spheres = zeros(nSpheres, (nSpherePts+1)^2, 3);
cylinderVol = pi * (cRadii.^2).*cHeights;
sphereVol = 4/3 * pi * (sRadius).^3;
cylinderSpeed = cylinderVol/2;
sphereSpeed = fallingSpeeds(vIdx); %sphereVol;
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


function val = isInImage(pts, idx, minX, maxX, minY, maxY)
    val = (pts(1, idx) >= minX) && (pts(2, idx) >= minY) ...
               && pts(1, idx) <= maxX && (pts(2, idx) <= maxY);
end


function points2d = projectPoints(points3d, P)
    points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
    points2dHomog = P * points3dHomog;
    points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end
