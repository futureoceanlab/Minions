function tracks = simulator2(params, simDir, simConfig, rndSeed)
    % SIMULATOR based on the stereo params and simulation configuration,
    % create 3D particles in 3D space. Then, project them to left and right
    % camera imaging plane. The particles move at every frame based on 
    % their speed configurations.
    
    %% Load camera parameters
    imageSizeY = 1944;
    imageSizeX = 2592;
    f = params.fl;
    N = params.N;
    WD = params.WD;
    pxPitch = params.pxPitch;    
    noiseVariances = simConfig.noiseVariances;    
    bgAvg = simConfig.bgAvg;
    bgAvg_scale1 = bgAvg/255;
    
    %% Path configuration
    imgDir = sprintf('%s/img', simDir);
    if ~exist(imgDir, 'dir')
        mkdir(imgDir)
    end
    
    for nIdx=1:length(noiseVariances)
        nDir = sprintf('%s/%d', imgDir, nIdx);
        if ~exist(nDir, 'dir')
            mkdir(nDir)
        end
    end
 
    %% Simulation configuration
    nFrames = simConfig.nFrames;
    nCylinderPts = 1e4; % Number of cloud points for a cylinder
    % Number of cloud points for a sphere
    nSpherePts = 20; 
    actualSpherePts = (nSpherePts+1)^2;
    tic

    cHeights = [];
    cRadius = [];
    sRadius = [];
    sSingleVelocity = [];
    cSingleVelocity = [];
    sRRange = diff(simConfig.sRadiusRange, [], 2);
    cRRange = diff(simConfig.cRadiusRange, [], 2);
    cHRange = diff(simConfig.cHeightsRange, [], 2);
    for pIdx = 1:size(simConfig.concentration, 1)
        c = round(simConfig.concentration(pIdx));
        % Compute number of spheres and cylinders based on their ratio
        nS = round(c * simConfig.sphereRatio);
        nC = c - nS;
        % assign random radius and sinking velocity within the provided
        % simulation range
        curSRadius = sRRange(pIdx)*rand([nS 1]) + simConfig.sRadiusRange(pIdx, 1);
        curCRadius = cRRange(pIdx)*rand([nC 1]) + simConfig.cRadiusRange(pIdx, 1);
        curCHeights = cHRange(pIdx)*rand([nC 1]) + simConfig.cHeightsRange(pIdx, 1);
        sRadius = [sRadius; curSRadius];
        cRadius = [cRadius; curCRadius];
        cHeights = [cHeights; curCHeights];
        curSVelocity = repmat(normrnd(simConfig.sinkRateDetail(pIdx, 1),...
            simConfig.sinkRateDetail(pIdx, 2), [nS, 1]), [1 3]);
        curCVelocity = repmat(normrnd(simConfig.sinkRateDetail(pIdx, 1),...
            simConfig.sinkRateDetail(pIdx, 2), [nC, 1]), [1 3]);
        sSingleVelocity = [sSingleVelocity; curSVelocity];
        cSingleVelocity = [cSingleVelocity; curCVelocity];
    end
    %  Convert from m to mm
    cHeights = cHeights./1000;
    cRadius = cRadius./1000;
    sRadius = sRadius./1000;
    nSpheres = size(sRadius, 1);
    nCylinders = size(cRadius, 1);
    nParticles = nSpheres + nCylinders;
    % Below is unnecessary since it is a linear move. We can simply
    % multiply the veclocity by frame number (i.e. time) to get the
    % displacement per each frame
    sSingleVelocity = abs(sSingleVelocity.*repmat(simConfig.direction, [nSpheres, 1]))./1000;
    cSingleVelocity = abs(cSingleVelocity.*repmat(simConfig.direction, [nCylinders, 1]))./1000;
    toc
    
    %% Generate and filter particles that do not fall in the image frame with buffer
    % Create simulated particles in 3D
    % origin is the center of the left
    % camera X-axis points right, Y-axis points bototm 
    centroids = generateCentroids(nParticles, -500, 500, -1500, 500, WD-250, WD+250);
    for cIdx = 1:length(centroids)
        centroids(cIdx, :) = (simConfig.R_fall)*(centroids(cIdx, :)');
    end
    
    % We need to filter out the particles that will not appear in the 
    % frame at all to reduce runtime of the software
    % To do so, we set boundaries in 3D space
    tracksL = table();
    tracksR = table();
    dof = params.dof;
    camBounds = [0 0 WD-dof; 2592 0 WD+dof; 0 1944 WD-dof; 2592 1944 WD+dof];
    leftBounds = project2Dto3D(camBounds, squeeze(params.camMatrix(1, :, :))');
    rightBounds = project2Dto3D(camBounds, squeeze(params.camMatrix(2, :, :))');
    minBound1 = min([leftBounds; rightBounds]);
    maxBound1 = max([leftBounds; rightBounds]);
    minBound1(3) = 200;
    singleVelocity = max([sSingleVelocity; cSingleVelocity]);
    % Based on the sinking velocity, find which particles will ever
    % appear in the frame
    bRange = nFrames * singleVelocity;
    minBound2 = minBound1 - bRange;
    maxBound2 = maxBound1 - bRange; 
    minBound = min([minBound1, minBound2]);
    maxBound = max([maxBound1, maxBound2]);
    cFilter = sum(centroids > minBound & centroids < maxBound, 2) == 3;
    % Filter out those particles
    centroids = centroids(cFilter, :);
    nParticles = size(centroids, 1);
    nSpheres = round(nParticles * simConfig.sphereRatio);
    nCylinders = nParticles - nSpheres;
    cSingleVelocity = cSingleVelocity(1:nCylinders, :);
    sSingleVelocity = sSingleVelocity(1:nSpheres, :);
    tic
    cSinkVelocity = permute(repmat(cSingleVelocity, [1 1 nCylinderPts]), [1 3 2]);
    sSinkVelocity = permute(repmat(sSingleVelocity, [1 1 actualSpherePts]), [1 3 2]);
    toc
    
    %% Generate 3D point clouds given the shape (cylinder + sphere) per centroids
    % Cylinders
    yawD = 0; 
    pitchD = 90*rand([nCylinders, 1]) - 45;
    rollD = 90*rand([nCylinders, 1]) - 45;
    cylinders = zeros(nCylinders, nCylinderPts, 3);
    spheres = zeros(nSpheres, actualSpherePts, 3);
    tic
    for i = 1:nParticles
        if i <= nCylinders
            cylinderPts = generateCylinder(cRadius(i), cHeights(i), nCylinderPts);
            cylinderPts = rotateCloud(cylinderPts, yawD, pitchD(i), rollD(i));
            cylinderPts = bsxfun(@plus, cylinderPts, centroids(i, :));
            cylinders(i, :, :) = cylinderPts;
        else
            k = i - nCylinders;
            spherePts = generateSphere(nSpherePts, sRadius(k));     
            spherePts = bsxfun(@plus, spherePts, centroids(i, :));
            spheres(i-nCylinders, :, :) = spherePts;
        end
    end
    toc
    
    
    %% Capture particles image and move frame by frame
    
    % Preprocessing related variables
    std1 = zeros(length(noiseVariances), 1);
    std2 = zeros(length(noiseVariances), 1);
    avg1 = zeros(length(noiseVariances), 1);
    avg2 = zeros(length(noiseVariances), 1);
    th1 = zeros(length(noiseVariances), 1);
    th2 = zeros(length(noiseVariances), 1);
    
    imgPair = zeros(2, imageSizeY, imageSizeX) + bgAvg;
    coord2D = zeros(2, nParticles, 2);
    coord3D = zeros(nParticles, 3);
    shapes = zeros(nParticles, 1);

    minSphere3D = squeeze(min(spheres, [], 2));
    maxSphere3D = squeeze(max(spheres, [], 2));
    minCylinder3D = squeeze(min(cylinders, [], 2));
    maxCylinder3D = squeeze(max(cylinders, [], 2));
    tic
    for frameI=1:nFrames
        % Filter out particles that will only fall in the current frame
        % First create the filter
        sBoundFilter = sum(minSphere3D < maxBound1 & maxSphere3D > minBound1, 2) == 3;
        cBoundFilter = sum(minCylinder3D < maxBound1 & maxCylinder3D > minBound1, 2) == 3;
        % retrieve the incies of the present particles, their point
        % clouds, their radii
        sIdx = find(sBoundFilter);
        cIdx = find(cBoundFilter);
        sInFrame = spheres(sBoundFilter, :, :);
        cInFrame = cylinders(cBoundFilter, :, :);
        cR = cRadius(cBoundFilter);
        cH = cHeights(cBoundFilter);
        sR = sRadius(sBoundFilter);
        % Compute number of the particles present in the current frame
        nSpheresInFrame = size(sInFrame, 1);
        nCylindersInFrame = size(cInFrame, 1);
        nParticlesInFrame = nSpheresInFrame + nCylindersInFrame;
        for i=1:nParticlesInFrame
            wasInFrame = [0 0];
            % Log the size of each particles (ESD)
            if (i <= nCylindersInFrame)
                curPts = squeeze(cInFrame(i, :, :));
                shapes(i) = 1;
                id = cIdx(i);
                esd = 2*nthroot((cR(i)^2)*cH(i)*3/4, 3); %mean([cR(i)*2 cH(i)]);
            else 
                k = i - nCylindersInFrame;
                curPts = squeeze(sInFrame(k, :, :));
                id = sIdx(k);
                esd = sR(k)*2;
            end
            coord3D(i, :) = mean(curPts)';
            % For each camera, project the 3D particle points onto the
            % image frame
            for camI=1:2
                imgTemp = zeros(imageSizeY, imageSizeX);
                dist = norm(coord3D(i, :) - params.camPoses(camI, :));
                if (abs(dist - WD) < params.dof)
                    pts = round(projectPoints(curPts, squeeze(params.camMatrix(camI, :, :))'))+1;
                    % Compute the circle of confusion (CoC)
                    COC = f * f / (N * (WD - f))* (abs(dist - WD) / dist)/pxPitch/2;
                    % Create a boundary around the particle, and check if
                    % it is in the frame
                    ptsMax = max(1, min([imageSizeX; imageSizeY], max(pts, [], 2) + floor(3*COC)));
                    ptsMin = min([imageSizeX; imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

                    if ~(inBoundary2D(ptsMin, 1, 2592, 1, 1944) ...
                            || inBoundary2D(ptsMax, 1, 2592, 1, 1944))
                        continue;
                    end
                    % When present in the frame, project 3D to 2D and 
                    % blur accordingly based on the CoC
                    coord2D(camI, i, :) = mean(pts, 2);
                    for j = 1:size(pts, 2)
                        x = pts(1, j);
                        y = pts(2, j);
                        if inBoundary2D(pts(:, j), 1, imageSizeX, 1, imageSizeY)
                            imgTemp(y, x) = 255;
                            wasInFrame(camI) = 1;
                        end
                    end
                    imgTemp2 = imgaussfilt(imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), 1.67*COC);
                    imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = imgTemp2;
                    imgPair(camI, :, :) = squeeze(imgPair(camI, :, :)) + imgTemp;
                end
            end
            % Log the particle information in the track table
            tracksL = updateTracks(tracksL, wasInFrame(1), id, ...
                squeeze(coord2D(1, i, :)), coord3D(i, :), esd);
            tracksR = updateTracks(tracksR, wasInFrame(2), id, ...
                squeeze(coord2D(2, i, :)), coord3D(i, :), esd);
        end
        % update the age
        if ~isempty(tracksL)
            tracksL.age = tracksL.age + 1;
        end
        if ~isempty(tracksR)
            tracksR.age = tracksR.age + 1;
        end
        % Image processing to mimic how the embedded board processes the
        % image
        lImg = squeeze(imgPair(1, :, :))./255;
        rImg = squeeze(imgPair(2, :, :))./255;
        for nIdx=1:length(noiseVariances)
            noise_var = noiseVariances(nIdx);            
            % On the first frame, we compute the average and noise variance
            % In the embedded board, we do this at least 50 times, but
            % this is a simulation.
            lImgN = imnoise(lImg, 'gaussian', bgAvg_scale1, noise_var);
            rImgN = imnoise(rImg, 'gaussian', bgAvg_scale1, noise_var);
            if frameI == 1
                std1(nIdx) = std(lImgN, 0, 'all');
                std2(nIdx) = std(rImgN, 0, 'all');
                avg1(nIdx) = sum(lImgN, 'all')/(2592*1944);
                avg2(nIdx) = sum(rImgN, 'all')/(2592*1944);
                th1(nIdx) = avg1(nIdx)+4*std1(nIdx);
                th2(nIdx) = avg2(nIdx)+4*std2(nIdx);
            end 
            lImgN(lImgN <= th1(nIdx)) = avg1(nIdx);
            rImgN(rImgN <= th2(nIdx)) = avg2(nIdx);
            % Save the images as TIFF
            img1Path = sprintf('%s/%d/cam_1_%d.tif', imgDir, nIdx, frameI);
            img2Path = sprintf('%s/%d/cam_2_%d.tif', imgDir, nIdx, frameI);
            imwrite(lImgN, img1Path, 'TIFF');
            imwrite(rImgN, img2Path, 'TIFF');
        end
        imgPair = zeros(2, imageSizeY, imageSizeX) + bgAvg;
        % Move the particles for the next frame
        cylinders = cylinders + cSinkVelocity;
        spheres = spheres + sSinkVelocity;
        minSphere3D = minSphere3D + sSingleVelocity;
        maxSphere3D = maxSphere3D + sSingleVelocity;
        minCylinder3D = minCylinder3D + cSingleVelocity;
        maxCylinder3D = maxCylinder3D + cSingleVelocity;
    end
    toc
    % Remove tracks that did not appear at all
    tracksL(tracksL.age == 0, :) = [];
    tracksR(tracksR.age == 0, :) = [];
    tracks = struct('tracksL', tracksL, 'tracksR', tracksR);
    trackFile = sprintf('%s/tracksSimulated.mat', simDir);
    save(trackFile, 'tracks');
end