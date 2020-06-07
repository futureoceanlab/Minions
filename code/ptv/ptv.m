clear;

lImg = imread('cam_1_1.tif', 'tiff');
rImg = imread('cam_2_1.tif', 'tiff');
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

%% Blob Detection
lImgBin = imbinarize(lImg);
rImgBin = imbinarize(rImg);


[areaL, lCentroids, lBbox] = blobAnalyzer(lImgBin);
[areaR, rCentroids, rBbox] = blobAnalyzer(rImgBin);

%% Epipolar Search on the right image
rPtsNew = zeros(size(lCentroids, 1), 4);
rRectNew = zeros(size(lCentroids, 1), 4);
for i=1:size(lCentroids, 1)
    lPts = [lCentroids(i, 1); lCentroids(i, 2); 1];
    pt3dLeft = (220.*inv(intrinsics) * lPts)';
    pt3dRight = (280.*inv(intrinsics) * lPts)'; 
    points = projectPoints([pt3dLeft; pt3dRight], cameraMatrix2');
    rPtsNew(i, :) = [points(1, :), points(2, :)];
end

%% Draw the result of epipolar lines
% figure; 
% hold on;
% imshow(cat(3, lImg, lImg, lImg));
% for i=1:size(lBbox, 1)
%     rectangle('Position', lBbox(i, :)', 'FaceColor', 'none', 'EdgeColor', 'red');
% end
% hold off;

% figure;
% hold on;
% imshow(cat(3, rImg, rImg, rImg));
% % for i=1:size(rBbox, 1)
% %     rectangle('Position', rBbox(i, :)', 'FaceColor', 'none', 'EdgeColor', 'blue');
% % end
% % line(rPtsNew(:,[1, 2])',rPtsNew(:,[3, 4])');
% hold off;
detectionID = 1;
tracks = [];
%% Kalmlan filter
%1. create new tracks
for i = 1:size(lCentroids, 1)
    centroid = lCentroids(i, :);
    area = areaL(i, :);
    bbox = lBbox(i, :);
    kalmanFilter = configureKalmanFilter('ConstantVelocity', centroid, [200, 50], [30, 25], 20);
    newTrack = table(i, {kalmanFilter}, 1, 1, 0, 0, ...
             255*rand(3, 1)', i, centroid, bbox, area, [NaN NaN NaN], [NaN NaN], 0, false,...
            'VariableNames', {'id', 'kalmanFilter', 'age',  'totalVisibleCount', ...
            'totalInvisibleCount', 'consecutiveInvisibleCount', 'colour', ...
            'currentLeftparticleId', 'centroid', 'bbox', 'area', ...
            'worldCoordinates', 'length', 'estimated', 'lost'
        });
    tracks = [tracks; newTrack];
end
predictedCentroids = [];
for i = 1:size(tracks, 1)
    predictedCentroid = predict(tracks.kalmanFilter{i});
    predictedCentroids = [predictedCentroids; predictedCentroid];
end
imColor = cat(3, lImg, lImg, lImg);
figure; 
hold on;
for i=1:size(predictedCentroids, 1)
    imColor(floor(predictedCentroids(i, 2)), floor(predictedCentroids(i, 1)), :) = [0, 0, 255];
end
imshow(imColor);
hold off;

%% projectPoints
function points2d = projectPoints(points3d, P)
    points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
    points2dHomog = P * points3dHomog;
    points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end