

%% Path configuration
stereoParamPath = 'params.mat'; %sprintf('%s/params.mat', stereoDataDir);
if ~exist(stereoDataDir, 'dir')
    fprintf('directory: %s does not exist! Creating... \n', simDataDir);
    return;
end
if ~isfile(stereoParamPath)
    fprintf('stereo parameter in %s does not exist! Quitting...\n', simDataDir);
    return;
end
load(stereoParamPath, 'params');

esdDir = 'esd_sim';


%% Load camera parameters
cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
    params.RotationOfCamera2, params.TranslationOfCamera2);
[~, lCameraPos] = extrinsicsToCameraPose(eye(3),...
                                        zeros(1, 3));
[~, rCameraPos] = extrinsicsToCameraPose(params.RotationOfCamera2,...
                                        params.TranslationOfCamera2);
camPoses = [lCameraPos; rCameraPos];
camMatrix = zeros([2, size(cameraMatrix1)]);
camMatrix(1, :, :) = cameraMatrix1;
camMatrix(2, :, :) = cameraMatrix2;
f1 = params.CameraParameters1.FocalLength;
pxPitch = 0.0022;
N = 8; % F-stop
fl = f1(1) * pxPitch;
desiredResolution = 0.020; %mm/px
WD = fl * desiredResolution/pxPitch;
rng(0,'twister');

imageSizeY = 1944;
imageSizeX = 2592;

%% Simulation  Configuration
noiseVariances = [0.000001 0.0004, 0.0025]; %, 0.01];
bgAvg = 35; % background average intensity
bgAvg_scale1 = bgAvg/255;

simMode = "monotonic";
sRadius = [25, 50, 100, 200]./1000; % minRadius to maxRadius (um)
depths = WD + [0; 5; 10; 15; 20; 25]; %  -25; -20; -15; -10; -5; 
nParticles = 25;

rndSeed = 0;
%% Blob Analyzer
boundBuffer = 25;

blobAnalyzer = vision.BlobAnalysis;
blobAnalyzer.MinimumBlobArea = 2;
blobAnalyzer.MaximumBlobArea = 20000;
blobAnalyzer.MaximumCount = 100;
blobAnalyzer.AreaOutputPort = true;
blobAnalyzer.CentroidOutputPort = true;
blobAnalyzer.BoundingBoxOutputPort = true;
blobAnalyzer.MajorAxisLengthOutputPort = true;
blobAnalyzer.MinorAxisLengthOutputPort = true;
blobAnalyzer.OrientationOutputPort = true;
blobAnalyzer.EccentricityOutputPort = true;
blobAnalyzer.EquivalentDiameterSquaredOutputPort = true;
blobAnalyzer.PerimeterOutputPort = true;

esdErr = zeros(length(sRadius), length(depths), length(noiseVariances));
esdErrStd = zeros(length(sRadius), length(depths), length(noiseVariances));
detectedParticles = zeros(length(sRadius), length(depths), length(noiseVariances));
SNRs = zeros(length(sRadius), length(depths), length(noiseVariances));
nDetectedParticles = zeros(length(sRadius), length(depths), length(noiseVariances));
nSimParticles = zeros(length(sRadius), length(depths), length(noiseVariances));
%% Monotonic Simulator 

% figure;
% hold on;
% title('Radial Profile');
% legendList = [];

kneeRadius = zeros(length(sRadius), length(depths), length(noiseVariances));
simCentroid = zeros(1, 2);

for rIdx=1:length(sRadius)

curSRadius = sRadius(rIdx);
esdPath = sprintf('%s/l_%d_%d_%d.tif', esdDir, curSRadius*1000, 1, round(depth));
if ~isfile(esdPath)
nSpherePts = 20*round(curSRadius/0.025);
actualSpherePts = (nSpherePts+1)^2;

spheres = zeros(nParticles, actualSpherePts, 3);
img = zeros(imageSizeY, imageSizeX);


for i = 1:nParticles
    spherePts = generateSphere(nSpherePts, curSRadius);     
    spherePts = bsxfun(@plus, spherePts, centroids(i, :));
    spheres(i, :, :) = spherePts;    
end
    imgPair = zeros(2, imageSizeY, imageSizeX) + bgAvg;
    for i=1:nParticles
        curPts = squeeze(spheres(i, :, :));

        for camI=1:1
            imgTemp = zeros(imageSizeY, imageSizeX);
            dist = norm(centroids(i, :) - (camPoses(camI, :) + [0 0 fl]));
            pts = round(projectPoints(curPts, squeeze(camMatrix(camI, :, :))'))+1;
            COC = (fl * fl / (N * WD)) * (abs(dist - WD) / dist)/pxPitch/2;
            ptsMax = max(1, min([imageSizeX; imageSizeY], max(pts, [], 2) + floor(3*COC)));
            ptsMin = min([imageSizeX; imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

            if ~(inBoundary2D(ptsMin, 1, 2592, 1, 1944) ...
                    || inBoundary2D(ptsMax, 1, 2592, 1, 1944))
                continue;
            end
%             for j = 1:size(pts, 2)
%                 x = pts(1, j);
%                 y = pts(2, j);
%                 if inBoundary2D(pts(:, j), 1, imageSizeX, 1, imageSizeY)
%                     imgTemp(y, x) = 255;
%                 end
%             end
            p = round(mean(pts, 2))
%             img(p(2), p(3)) = 255;
%             imgTemp2 = imgaussfilt(imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), COC);
%         %                 imgTemp2 = imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1));
%             imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = imgTemp2;
%             imgPair(camI, :, :) = squeeze(imgPair(camI, :, :)) + imgTemp;
        end
    end
    lImg = squeeze(imgPair(1, :, :))./255;
    rImg = squeeze(imgPair(2, :, :))./255;
end
for dIdx=1:length(depths)
depth=depths(dIdx);
centroids = generateCentroidsFixedDist(nParticles, -30, 20, -20, 15, depth);

formalCOC = fl * fl / (N * (WD - fl))* abs((depth - WD-fl) / (depth-fl))/pxPitch/2;

% figure;
% hold on;
% title(sprintf('Radial Profile for ESD=%d', curSRadius*2000));
% legendList = [];
% xline(curSRadius/0.02, 'HandleVisibility','off')


    for nIdx = 1:length(noiseVariances)
        noiseVar = noiseVariances(nIdx);
        newEsdPath = sprintf('%s/l_%d_%d_%d.tif', esdDir, curSRadius*1000, nIdx, round(depth));

        if ~isfile(newEsdPath)
            lImgN = imnoise(lImg, 'gaussian', bgAvg_scale1, noiseVar);
            imwrite(lImgN, newEsdPath, 'TIFF');
            lImgN = lImgN/255;
%             rImgN = 255*imnoise(rImg, 'gaussian', bgAvg_scale1, noiseVar);
        else
            lImgN = double(imread(newEsdPath));
        end
%         PSF = fspecial('gaussian',5,5);
%         luc1 = deconvlucy(lImgN,PSF,5);
%         figure; imshow(luc1);
        var1 = var(lImgN, 0, 'all');
        avg1 = sum(lImgN, 'all')/(2592*1944);
        th1 = avg1+sqrt(var1)*3.5;
%         COC = fl * fl / (N * (WD - fl))* (abs(depth - WD) / depth)/pxPitch/2
        img1 = imgaussfilt(lImgN, 2);
        img1(img1 <= th1) = 1;
        logImg1 = rescale(log2(img1));
        binImg1 = imbinarize(logImg1); 
        [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(binImg1);
        boundaryFilter = centroid(:, 1) < boundBuffer |centroid(:, 2) < boundBuffer ...
            | centroid(:, 1) > 2590-boundBuffer | centroid(:, 2) > 1944 - boundBuffer;
                    mean2(lImgN - avg1);
        nFound = size(centroid, 1);
        if rIdx == 1 && dIdx ==1 && nIdx == 1
            simCentroid = centroid(1, :);
        end
        if nFound >= 1
            scanRange = 25;
            rIn = zeros(size(centroid, 1), scanRange);

            signalEx = zeros(size(centroid, 1), 1);
            for k=1:size(centroid, 1)
                c = round(centroid(k, :));
                rIn(k, :) = computeRadialIntensity(scanRange, c, lImgN);
%                 count = 0;
%                 pxSum = 0;
%                 for x=bbox(k, 1):(bbox(k, 1)+bbox(k, 3))
%                     for y=bbox(k, 2):(bbox(k, 2)+bbox(k, 4))
%                         if ((x-c(1))^2+(y-c(2))^2) <= (sqrt(r2(k))/2)^2
%     %                         lImgN(y, x) = 255;
%                             pxSum = pxSum + lImgN(y, x);
%                             count = count + 1;
%                         end
%                     end
%                 end
%                 if count == 0
%                     continue;
%                 end
%                 signalEx(k) =  pxSum / count;
            end
            if (size(rIn, 1) == 1)
                rInMean = rIn;
            else
                rInMean = mean(rIn);
            end
            maxIn = mean(rInMean(1:2));
%             kneeStart = (0.9977)*(maxIn-avg1)+avg1;
            knee = (0.242)*(maxIn-avg1)+avg1; % 95
%             [~, kneeStartIdx] = min(abs(rInMean-kneeStart));
            [~, kneeIdx] = min(abs(rInMean-knee));
            actualESD = kneeIdx - formalCOC;
            actualESDIntensity =rInMean(floor(actualESD))-( rInMean(floor(actualESD)) - rInMean(ceil(actualESD)))*(actualESD-floor(actualESD));
%             if (rInMean(kneeStartIdx) > kneeStart)
%                 kneeStartIdx = kneeStartIdx + 1;
%             end
%             if (rInMean(kneeIdx) > knee)
%                 kneeIdx = kneeIdx + 1;
%             end
            kneeRadius(rIdx, dIdx, nIdx) = kneeIdx-1-formalCOC;
            px = 0:(scanRange-1);
%             plot(px(1:end-1), diff(rInMean));
% % %             plot(px(kneeStartIdx), rInMean(kneeStartIdx), 'b*');
%             plot(px(kneeIdx)-formalCOC, actualESDIntensity, 'b*', 'HandleVisibility','off');
%             plot(px(kneeIdx), rInMean(kneeIdx), 'r*', 'HandleVisibility','off');
% % %             legendList = [legendList, sprintf("n=%.4f", noiseVar)];
%             legendList = [legendList, sprintf("esd=%d, dist=%.1f", curSRadius*2000, depth-WD-fl)];
            signalEx = signalEx - avg1;
%             SNR = rInMean(1:ceil(curSRadius/20))/sqrt(var1); signalEx / sqrt(var1);
%             mean(SNR)
            id = 1:size(centroid, 1);
            lParticles = table(id', area, centroid, bbox, ...
                                ax_mj, ax_mn, or, ecc, r2, per);
            nDetectedParticles(rIdx, dIdx, nIdx) = height(lParticles);

            lParticles.Properties.VariableNames{1} = 'id';
            lParticles.area = double(lParticles.area);
            lParticles(boundaryFilter, :) = [];
            esd = sqrt(lParticles.r2)*20; %-((2*ceil(2*formalCOC)+1)*mean(SNR));
            avgESD = mean(esd);
            stdESD = sqrt(var(esd));
            diffESD = abs(avgESD - curSRadius*2000);
            stdDiffESD = sqrt(var(esd - curSRadius*2000));
            esdErr(rIdx, dIdx, nIdx) = diffESD;
            esdErrStd(rIdx, dIdx, nIdx) = stdDiffESD;
            detectedParticles(rIdx, dIdx, nIdx) = height(lParticles);
            SNRs(rIdx, dIdx, nIdx) = mean(rInMean(1:2))/sqrt(var1); %mean(SNR);
%             figure;
%         hold on;
%         imshow(lImgN, [0 255]);
%         title(sprintf("%d %d", curSRadius*1000, depth));
%         for j=1:height(lParticles)
%             rectangle('Position', lParticles.bbox(j, :), 'EdgeColor', 'red');
%         end
%         hold off;
%         close all;
        else
            simIntensity = computeRadialIntensity(scanRange, round(simCentroid), lImgN);
            SNRs(rIdx, dIdx, nIdx) = mean(simIntensity(1:2))/sqrt(var1);
        end

        %         J = rescale(J, 0, 255);
        %         imshow(J, [0 255]);

        % img1Path = sprintf('%s/cam_1.tif', imgDir);
        % img2Path = sprintf('%s/cam_2.tif', imgDir);
        %         img1PathBin = sprintf('%s/cam_1_%d.bin', imgDir, frameI);
        %         fd= fopen(img1PathBin, 'w');
        %         fwrite(fd, lImgN);
        %         fclose(fd);
        % imwrite(lImgN, img1Path, 'TIFF');
        % imwrite(rImgN, img2Path, 'TIFF');
    end

%     close all;

end
% legend(legendList);
% 
% hold off;

end

% figure; hold on; title('ESD=50'); plot(depths-WD-fl, 25-squeeze(nDetectedParticles(1, :, :))); legend(["{\sigma}=0.001", "{\sigma}=0.02", "{\sigma}=0.05"]); ylabel("# missed detection"); xlabel("depth offset (mm)"); xlim([0 25]); ylim([-1 25]);
% figure; hold on; title('ESD=100'); plot(depths-WD-fl, 25-squeeze(nDetectedParticles(2, :, :))); legend(["{\sigma}=0.001", "{\sigma}=0.02", "{\sigma}=0.05"]); ylabel("# missed detection"); xlabel("depth offset (mm)"); xlim([0 25]); ylim([-1 25]);
% figure; hold on; title('ESD=200'); plot(depths-WD-fl, 25-squeeze(nDetectedParticles(3, :, :))); legend(["{\sigma}=0.001", "{\sigma}=0.02", "{\sigma}=0.05"]); ylabel("# missed detection"); xlabel("depth offset (mm)"); xlim([0 25]); ylim([-1 25]);
% figure; hold on; title('ESD=400'); plot(depths-WD-fl, 25-squeeze(nDetectedParticles(4, :, :))); legend(["{\sigma}=0.001", "{\sigma}=0.02", "{\sigma}=0.05"]); ylabel("# missed detection"); xlabel("depth offset (mm)"); xlim([0 25]); ylim([-1 25]);
for rIdx=1:length(sRadius)
    figure;
    data = squeeze(esdErr(rIdx, :, :));
    model_error = squeeze(esdErrStd(rIdx, :, :));
%     yyaxis left
    b = bar(depths, data, 'grouped');

    hold on
    title(sprintf('PTV error at sphere ESD %d (um)', sRadius(rIdx)*2000));
    % Calculate the number of bars in each group
    nbars = size(data, 2);
    % Get the x coordinate of the bars
    x = [];
    for i = 1:nbars
        x = [x ; b(i).XEndPoints];
    end
    % Plot the errorbars
    errorbar(x',data,model_error,'k','linestyle','none');
%     missedParticles = nParticles-squeeze(detectedParticles(rIdx, :, :));
%     yyaxis right
%     p = plot(depths, squeeze(SNRs(rIdx, :, :)));
%     yyaxis left
    ylabel('ESD Calculation Error (um)')
%     yyaxis right
%     ylabel('No. Missed Particles')
    hold off
    
end
