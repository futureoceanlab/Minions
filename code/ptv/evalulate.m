% D_EVALUATE meant for diverse particle setup evaluation
% We run PTV based on the camera parameters and compared the result
% to groundtruth simulation to analyze the performance

%% Simulation parameters configurations
simMode = "diverse";
% particle detection mode, global threshold uses Otsu's method,
% adaptive is a local detection mode, which is not robust against a 
% noisy image
detectionMode = "global"; %"adaptive";
% particles within boundBuffer from the image exterior boundary are not 
% considered in blob detection
boundBuffer = 25;
% value for assigning particle to a new track in Munkres
costOfNonAssignment = 30;
% value for lost trace determination
visibilityRatio = 0.6;

%% Path configuration
stereoDataDir = sprintf('data/stereocamera_%d', 3);

stereoParamPath = sprintf('%s/params.mat', stereoDataDir);
if ~exist(stereoDataDir, 'dir')
    fprintf('directory: %s does not exist! Creating... \n', simDataDir);
    return;
end
if ~isfile(stereoParamPath)
    fprintf('stereo parameter in %s does not exist! Quitting...\n', simDataDir);
    return;
end

load(stereoParamPath, 'params');

camPoses = params.camPoses;
desiredResolution = 0.020; % mm/px
imageSizeY = 1944;
imageSizeX = 2592;
focalLen = params.fl;
N = params.N; % F-stop number
WD = params.WD; % working distance
pxPitch = params.pxPitch;

simDataDir = sprintf('%s/new_simulation_preprocessed_norm_concentration_sig4', stereoDataDir);
listing = dir(simDataDir);

trackDir = sprintf('%s/tracks', simDataDir);

nNoiseLevels = 4; % number of noise level configurations
nFallDir = 2; % number of falling direction configurations
nComb = nNoiseLevels*nFallDir; % total number of configurations

% 2D and 3D error in x, y, and z directions
%   errors in the first row
%   STD in the second row
trackStats2D = zeros(2, nComb, 2);
trackStats3D = zeros(2, nComb, 3);
% number of 3D and 2D particles detected in each configuration
n3D = zeros(nComb, 1);
n2D =  zeros(nComb, 1);
% number of false-positives
falsePositive = zeros(nComb, 1);
fileIdx = 1;

% Iterate each configuration
nParticlesDetected = zeros(length(listing)-2, 2);
for subPathIdx=1:length(listing)
    %% Read relevant 'diverse' dataset only
    dirItem = listing(subPathIdx);
    targetFile = startsWith(dirItem.name, "d_data");
    if ~(dirItem.isdir && targetFile)
        continue;
    end
    tic
    dirItem.name;
    %% Load simulation related configuration and data information
    curSimDataDir = sprintf("%s/%s", simDataDir, dirItem.name);
    curSimImgDir = sprintf("%s/img", curSimDataDir);
    simConfigPath = sprintf("%s/config.mat", curSimDataDir);
    simTracksPath = sprintf("%s/tracksSimulated.mat", curSimDataDir);
    curTrackDir = sprintf("%s/%s", trackDir, dirItem.name);
    if ~exist(curTrackDir, 'dir')
        mkdir(curTrackDir);
    end
    load(simConfigPath, 'curSimConfig');
    nFrames = curSimConfig.nFrames;
    

    %% PTV
    % Check if PTV result already exists
    for nIdx=1:nNoiseLevels
        nImgPath = sprintf('%s/%d', curSimImgDir, nIdx);
        curNTrackDir = sprintf('%s/%d', curTrackDir, nIdx);
        if ~exist(curNTrackDir, 'dir')
            mkdir(curNTrackDir);
        end
        computedTrackPath = sprintf('%s/tracksComputed_%d_%s_border_%d.mat', ...
        curNTrackDir, costOfNonAssignment, detectionMode, boundBuffer);
        if ~isfile(computedTrackPath)
            runPTV(params, nFrames, nImgPath, computedTrackPath, ...
                costOfNonAssignment, detectionMode, boundBuffer, WD-30, WD+30, visibilityRatio);
        end
    
    
    %% Setup for evaluation
    curTrackEvalDir = sprintf("%s/evaluation", curNTrackDir);
    if ~exist(curTrackEvalDir, 'dir')
        mkdir(curTrackEvalDir);
    end
    load(simTracksPath, 'tracks');
    
    % simulation tracks
    sL = tracks.tracksL;
    sR = tracks.tracksR;
    sL = sortrows(sL, 'id');
    sR = sortrows(sR, 'id');
    % Filter out invalid tracks
    lostThreshold = 25;
    sL(sL.totalVisibleCount < lostThreshold, :) = [];
    sR(sR.totalVisibleCount < lostThreshold, :) = [];
    
    % computed tracks
    load(computedTrackPath, 'tracks');
    tL = tracks.tracksL;
    tR = tracks.tracksR;
    tL = sortrows(tL, 'id');
    tR = sortrows(tR, 'id');

    %% 1. Match PTV tracks to simulation
    img1Path = sprintf('%s/cam_1_1.tif', nImgPath);
    img2Path = sprintf('%s/cam_2_1.tif', nImgPath); %curSimImgDir);
    img1 = imread(img1Path, 'TIFF');
    img2 = imread(img2Path, 'TIFF');
    % uncomment to see the trace figures
%     figTrace = drawTraceFigure(tL, img1, 'Particle Trace on the Left Camera');
%     pause(1);
%     figTrace2 = drawTraceFigure(tR, img2, 'Particle Trace on the Right Camer');
    [trackMatch1, matchIdx1] = matchTracksPTVtoSim(tL, sL, nFrames);
    [trackMatch2, matchIdx2] = matchTracksPTVtoSim(tR, sR, nFrames);
    
    %% 2. Comparison against simulations
    [count2DL, err2DL, trackMatch1, missedSimL] = compare2DResults(tL, sL, trackMatch1, matchIdx1, camPoses(1, :));
    [count2DR, err2DR, trackMatch2, missedSimR] = compare2DResults(tR, sR, trackMatch2, matchIdx2, camPoses(2, :));
    err2D = [err2DL; err2DR];
    %% Analyse the computed particles that were missed in simulation
    vL = zeros(height(missedSimL), 5);
    vR = zeros(height(missedSimR), 5);
    % Right
    for q=1:height(missedSimR)
        % ESD
        vR(q, 1) = missedSimR.esd(q);
        % Distance from the working distance
        vR(q, 2) = max(abs(WD - (vecnorm(missedSimR.trace{q}.detectedTrace3D - squeeze(camPoses(2, :, :)), 2, 2))));
    end
    % Left
    for q=1:height(missedSimL)
        % ESD
        vL(q, 1) = missedSimL.esd(q);
        % Distance from the working distance
        vL(q, 2) = WD - max(vecnorm(missedSimL.trace{q}.detectedTrace3D - squeeze(camPoses(1, :, :)), 2, 2));
        % Error 
%         vL(q, 3:5) = mean(diff(missedSimR.trace{q}.detectedTrace3D));
%         if (sum(vL(:, 1) == vR(q, 1)) > 0)
%             vL(q, 1) = 0;
%             vL(q, 2) = 0;
%         end
    end
    
    %% Take a look at the simulation particles 
    vvL = zeros(height(sL), 2);
    vvR = zeros(height(sR), 2);
    for q=1:height(sL)
        vvL(q, 1) = sL.esd(q);
        vvL(q, 2) = max(abs(WD - (vecnorm(sL.trace{q}.detectedTrace3D - squeeze(camPoses(1, :, :)), 2, 2))));
    end
    
    for q=1:height(sR)
        vvR(q, 1) = sR.esd(q);
        vvR(q, 2) = max(abs(WD - (vecnorm(sR.trace{q}.detectedTrace3D - squeeze(camPoses(2, :, :)), 2, 2))));
    end
%     vL((sum(vL == [0 0], 2) == 2), :) = [];
%     vMissing = [vR; vL];
    %% 3D Analysis
    [count3D, err3D, falsePos] = compare3DResults(tL, sL, trackMatch1, matchIdx1);
    
    % Find how many of simulation particles should be 3D trackable
    numSim3D = find3DSim(params, sL);
    % Find total number of detected particles in left or right cameras
    numSim2D = height(sL) + height(sR) - numSim3D;
    % Find computed number of detected particles in 2D
    numAct2D = count2DL + count2DR - count3D;
    
    resultIdx = (fileIdx-1)*nNoiseLevels+nIdx;
    n2D(resultIdx) = numAct2D;
    trackStats2D(1, resultIdx, :) = mean(err2D);
    trackStats2D(2, resultIdx, :) = std(err2D);

    n3D(resultIdx) = count3D;
    trackStats3D(1, resultIdx, :) = mean(err3D);
    trackStats3D(2, resultIdx, :) = std(err3D);
    falsePositive((fileIdx-1)*4+nIdx) = falsePos; 
    
    toc
    end
    fileIdx = fileIdx + 1;  
end