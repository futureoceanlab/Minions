% ANALYZE DETECTABILITY
% We are interested in learning about the detectability of particles
% at varying depth given their ESD. We expect the smaller particles 
% to be less visible under noise, and therefore, their effective imaging
% volume is smaller. This is important when computing the concentration.

% Configuration
esdDir = 'esd_sim2';
params = createStereoParams();
imageSizeY = 1944;
imageSizeX = 2592;
f = params.fl; % focal length
N = params.N; % f-stop
WD = params.WD; % Working distance
% number of particles visible to be present
nParticles = 16;
pxPitch = params.pxPitch;
bgAvg = 0.1;
intrinsics1 = params.CameraParameters1.IntrinsicMatrix;
R1 = eye(3);
T1 = zeros(1, 3);
R2 = params.RotationOfCamera2;         
T2 = params.TranslationOfCamera2;

%% 3D particle simulation configuration
% depths at which the particles will be placed
depths = WD + [-30; -25; -20; -15; 10; -5; 0; 5; 10; 15; 20; 25; 30];
sRadiusRange = [25 50; 50 75; 75 125; 125 195; 195 320; 320 520; 520 850; 850 1400];
sRadius = mean(sRadiusRange, 2)./1000;
% coordinates of each particles for use later
pts2D = zeros(nParticles, 2);
pts3D = zeros(nParticles, 3);

% We place the particles evenly spread out in the imaging frame
% They will be projected onto 3D space later
wUnit = imageSizeX/sqrt(nParticles)/2;
hUnit = imageSizeY/sqrt(nParticles)/2;
for x=0:3
    w = wUnit * (x*2+1);
    for y=0:3
        h = hUnit*(y*2+1);
        pIdx = 4*x + y+1;
        pts2D(pIdx, :) = [h, w];
    end
end

%% Create images
% Make images for every radius and depth
for rIdx=1:length(sRadius)
    curSRadius = sRadius(rIdx);
    % Setup for 3D sphere coordinates
    nSpherePts = round(max(sRadius(rIdx)/0.02, 4) * 5);
    actualSpherePts = (nSpherePts+1)^2;
    spheres = zeros(nParticles, actualSpherePts, 3);
    
    for dIdx = 1:size(depths, 1)
        depth = depths(dIdx);
        esdPath = sprintf('%s/cam_%d_%d.tif', esdDir, round(curSRadius*1000), round(depth-WD));
        if isfile(esdPath)
            % the image already exists, skip
            continue;
        end
        
        img = zeros(imageSizeY, imageSizeX);

        for pIdx=1:nParticles
            % project 2D coordinate to 3D at certain depth
            pts2D_H = [pts2D(pIdx, 2), pts2D(pIdx, 1), 1];
            temp3D = depth.*inv(intrinsics1')*(pts2D_H');
            pts3D(pIdx, :) = R1*(temp3D - T1');
            vDist = pts3D(pIdx, :) - params.camPoses(1, :);
            dist = norm(vDist);
            pts3D(pIdx, :) = pts3D(pIdx, :) + vDist*(1-dist/WD);
            % Generate sphere
            sPoints = generateSphere(nSpherePts, curSRadius);     
            sPoints = bsxfun(@plus, sPoints, pts3D(pIdx, :));
            
            % Project only the visible region of the sphere onto the camera
            % to reduce running time
            c = pts3D(pIdx, :);
            % define the normal to the plane of interest
            fcLine = params.camPoses(1, :) - c; 
            fcDist = norm(fcLine);
            normFn = fcLine./fcDist;
            interPoint = c + normFn*(curSRadius^2/fcDist);
            
            vPoints = zeros(size(sPoints));
            for pIdx =1:size(sPoints)
                visible = dot(normFn, sPoints(pIdx, :)-interPoint) > 0;
                vPoints(pIdx, :) = visible*sPoints(pIdx, :);
            end
            vPoints(sum(vPoints==[0 0 0], 2)==3, :) = [];
            
            % project the 3D points onto the 2D plane
            pts = round(projectPoints(vPoints, squeeze(squeeze(params.camMatrix(1, :, :)))'))+1;
            % compute the appropriate CoC based on the depth
            COC = (fl * fl / (N * WD)) * (abs(dist - WD) / dist)/pxPitch/2;
            ptsMax = max(1, min([imageSizeX; imageSizeY], max(pts, [], 2) + floor(3*COC)));
            ptsMin = min([imageSizeX; imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

            for j = 1:size(pts, 2)
                x = pts(1, j);
                y = pts(2, j);
                if inBoundary2D(pts(:, j), 1, imageSizeX, 1, imageSizeY)
                    img(y, x) = 255;
                end
            end
            % blur
            imgTemp = imgaussfilt(img(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), 1.67*COC);
            img(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = imgTemp./255;
        end
        imwrite(img, esdPath);
    end
end

%% Detect particles based on a Blob Analyzer

% noise levels
nLevels = (0.001:0.002:0.05).^2;

LostStd = zeros(length(sRadius), size(depths, 1));
SNRs = zeros(length(sRadius), size(depths, 1));
for rIdx=1:length(sRadius)
    curSRadius = sRadius(rIdx);
    for dIdx = 1:size(depths, 1)
        depth = depths(dIdx);
        esdPath = sprintf('%s/cam_%d_%d.tif', esdDir, round(curSRadius*1000), round(depth-WD));
        if ~isfile(esdPath)
            continue;
        end
        img = double(imread(esdPath))./255;
        for nIdx=1:size(nLevels, 2)
            % Apply noise to the image
            noise_var = nLevels(nIdx);
            imgNOrig = imnoise(img, 'gaussian', bgAvg, noise_var);
            imgN = imgNOrig*255;
            % mimic thresholding in the actual embedded board
            stdN = std(imgN, 0, 'all');
            avgN = sum(imgN, 'all')/(imageSizeX * imageSizeY);
            thN = avgN+4*stdN;
            imgN(imgN <= thN) = avgN;
            
            imgFilt = imgaussfilt(imgN, 2);
            img1 = rescale(log2(imgFilt));
            % Detect particles!
            particles = detectParticles(img1, 0);
            
            % If we did not find exactly 16 particles, we consider
            % this depth to be invalid
            if height(particles) ~= 16
                % Remember the last noise level of valid detection
                LostStd(rIdx, dIdx) = nIdx-1;
                % Hmm, not the best thing to do
                % but I decided to re-run the detection on the last
                % noise level to compute the SNR
                noise_var = nLevels(nIdx-1);
                imgNOrig = imnoise(img, 'gaussian', bgAvg, noise_var);
                imgN = imgNOrig*255;
                stdN = std(imgN, 0, 'all');
                avgN = sum(imgN, 'all')/(imageSizeX * imageSizeY);
                thN = avgN+4*stdN;
                imgN(imgN <= thN) = avgN;

                imgFilt = imgaussfilt(imgN, 2);
                img1 = rescale(log2(imgFilt));
                particles = detectParticles(img1, 0);
                scanRange = round(curSRadius/0.02);
                centroid = particles.centroid;
                rIn = zeros(size(centroid, 1), scanRange);
                for k=1:size(centroid, 1)
                    c = round(centroid(k, :));
                    rIn(k, :) = computeRadialIntensity(scanRange, c, imgNOrig*255);
                end
                rInMean = mean(mean(rIn)) - avgN;
                SNR = rInMean/stdN;
                SNRs(rIdx, dIdx) = SNR;
                % We ignore the rest of the noise level, which are onlyu
                % higher
                break
            end
        end
    end    
end


%% Compute the effective volume in which the particles were detectable
depthsM = -30:5:0;
depthsP = 5:5:30;
% compute overlapping volume if first time
% v = zeros(size(depthsM, 2), size(depthsP, 2));
% for mIdx=1:size(depthsM, 2)
%     mDepth =depthsM(mIdx);
%     for pIdx = 1:size(depthsP, 2)
%         pDepth = depthsP(pIdx);
%         [v(mIdx, pIdx), ~] = computeOverlapVol(B, th, fl, mDepth, pDepth, 0);
%     end
% end

load('overlapV.mat', 'v');
% [minDepth, maxDepth, volume, interpolationFlag]
% interpolationFlag: 1 if the first undetectable depth
nsdChart = zeros(length(sRadius), size(nLevels, 2), 4);
nsdInterp = zeros(length(sRadius), size(nLevels, 2));
for sIdx=1:length(sRadius)
    prevMin = 1;
    prevMax = size(depthsP, 2);
    nsdChart(sIdx, 1, 4) = 1;
    for nIdx=1:size(nLevels, 2)
        nLevel = sqrt(nLevels(nIdx));
        idx0 = size(depthsM, 2)+1;
        % check if there was any undetectable depth on the front part 
        % of the volume
        minDExists = find(LostStd(sIdx, 1:idx0) == nIdx, 1);
        % back part of the volume
        maxDExists = find(LostStd(sIdx, (idx0+1):end) == nLevel, 1, 'last');
        if ~isempty(minDExists)
            nsdChart(sIdx, nIdx, 1) = minDExists; 
            nsdChart(sIdx, nIdx, 4) = 1;
            prevMin = minDExists;
        else
            % We assume that higher noise would have at least the 
            % invalid depth the previous invalid depth
            nsdChart(sIdx, nIdx, 1) = prevMin;
        end
        if ~isempty(maxDExists)
            nsdChart(sIdx, nIdx, 2) = maxDExists;
            nsdChart(sIdx, nIdx, 4) = 1;
            prevMax = maxDExists;
        else
            nsdChart(sIdx, nIdx, 2) = prevMax;
        end
        if (nIdx == size(nLevels, 2))
            nsdChart(sIdx, nIdx, 4) = 1;
        end
        % Assign right volume
        nsdChart(sIdx, nIdx, 3) = v(squeeze(nsdChart(sIdx, nIdx, 1)), squeeze(nsdChart(sIdx, nIdx, 2)));
    end
    % Interpolation
    N = 1:size(nLevels, 2);
    nFilter = squeeze(nsdChart(sIdx, :, 4));
    if ~isempty(nFilter)
        x = N(nFilter == 1);
        vx = squeeze(nsdChart(sIdx, x, 3));
        d = interp1(x, vx, N);
        nsdInterp(sIdx, :) = d;
    else
        nsdInterp(sIdx, :) = squeeze(nsdChart(sIdx, :, 3));
    end
end
legendList = ["ESD=75µm", "ESD=125µm", "ESD=200µm", "ESD=320µm"];
figure; 
hold on;
title("Noise vs Relative Imaging Volume per ESD", 'FontSize', 12);
plot(nLevels, nsdInterp(1:4, :)', 'LineWidth', 2);
xlabel("Noise Level (Std. Dev.)");
ylabel("Relative Imaging Volume, %");
ylim([0 100]);
grid on;
legend(legendList);