% clear;
load("params.mat", 'params');
dataDir = "good_photos_undistor8";
outputDir = "output";
numImages = 10;
nx = 26;
ny = 26;
width = 2592; % width
height = 1944; % height 
nCorners = nx * ny;
formatSpec = '%f\t%f';
sizeA = [2 nCorners];
imagePoints = zeros(nCorners, 2, numImages , 2);
leftImages = [];
rightImages = [];
for i=1:2
   for j=1:numImages
        filePath = sprintf("%s/%s/cam_%d_%d.txt",...
            dataDir, outputDir, i, j);
        fileID = fopen(filePath,'r');
        A = fscanf(fileID,formatSpec,sizeA);
        imagePoints(:, :, j, i) = A';
        fclose(fileID);
   end
end
for k=1:numImages
   lPoints = imagePoints(:, :, k, 1);
   rPoints = imagePoints(:, :, k, 2);
   [point3d5, r5] = triangulate(lPoints, rPoints, params);
   [mean(r5) std(r5)];
   scatter3( point3d5(:, 3), point3d5(:, 1), point3d5(:, 2), '.');
end