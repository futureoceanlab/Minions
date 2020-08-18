% MATLAB stereo calibration based on the grid points that were detected
% by the C++ code in the same directory. 

% calibration path and configurations
dataDir = "water-stereo";
outputDir = "output";
numImages = 6;
% number of points along x- and y-axes. 
nx = 26;
ny = 26;
width = 2592; % px
height = 1944; % px
nCorners = nx * ny;
% for string formatting later
formatSpec = '%f\t%f';
sizeA = [2 nCorners];
imagePoints = zeros(nCorners, 2, numImages , 2);
leftImages = [];
rightImages = [];
% Parse detected points data from individual txt files stored from
% the C++ software
for i=1:2 % each camera
   for j=1:numImages % each image
        filePath = sprintf("%s/%s/cam_%d_%d.txt",...
            dataDir, outputDir, i, j);
        imagePath = sprintf("%s/cam_%d_%d.png",...
            dataDir, i, j);
        % The following if case is to store the image path
        % for future visualization of the images
        if i == 1
            rightImages = [rightImages imagePath];
        else
            leftImages = [leftImages imagePath];
        end
        fileID = fopen(filePath,'r');
        A = fscanf(fileID,formatSpec,sizeA);
        % read all of the image grid points
        imagePoints(:, :, j, i) = A';
        fclose(fileID);
   end
end

squareSize = 1; %mm
worldPoints = generateCheckerboardPoints([nx+1 ny+1], squareSize);
imageSize = [height width];
params = estimateCameraParameters(imagePoints, worldPoints, ...
                                  'ImageSize',imageSize);
%% Show reprojetion error
% figure;
% showReprojectionErrors(params);

%% Show Extrinsics
figure;
ax = showExtrinsics(params);
ax.Title.FontSize = 12;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
leftImds = imageDatastore(leftImages);
rightImds = imageDatastore(rightImages);

%% Visualization of undistorted images + triangulate analysis
% for i=1:10
%     I1 = rgb2gray(readimage(leftImds, 1));
%     I2 = rgb2gray(readimage(rightImds, 1));
%     I1 = undistortImage(I1, params.CameraParameters1);
%     I2 = undistortImage(I2, params.CameraParameters2);
%     imgTitle1 = sprintf("cam_%d_%d.png", 1, i);
%     imgTitle2 = sprintf("cam_%d_%d.png", 2, i);
%     imwrite(I1, imgTitle1);
%     imwrite(I2, imgTitle2);
% end
% % figure; imshow(I1);
% % figure; imshow(I2);
% [point3d1, r1] = triangulate([161, 513], [369, 274], params)
% % [point3d2, r2] = triangulate([1389, 500], [1486, 291], params)
% % [point3d3, r3] = triangulate([1312, 378], [1358, 172], params)
% % [point3d4, r4] = triangulate([1309, 1346], [1334, 1588], params)
% [point3d5, r5] = triangulate([686, 127], [361, 358], params)
% [J1_valid,J2_valid] = rectifyStereoImages(I1, I2, params, 'OutputView', 'Full');
% % [J1_valid,J2_valid] = rectifyStereoImages(I1,I2,params, ...
% %   'OutputView','valid');
% figure; imshow(J1_valid + J2_valid);
