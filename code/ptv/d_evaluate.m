clear;
%% Simulation parameters configurations
simMode = "diverse";
detectionMode = "adaptive";
costOfNonAssignment = 25;
boundBuffer = 25;
%% Path configuration
stereoDataDir = sprintf('data/stereocamera_%d', 1);

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
cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
    params.RotationOfCamera2, params.TranslationOfCamera2);
[~, lCameraPos] = extrinsicsToCameraPose(eye(3),...
                                        zeros(1, 3));
[~, rCameraPos] = extrinsicsToCameraPose(params.RotationOfCamera2,...
                                        params.TranslationOfCamera2);
camPoses = [lCameraPos; rCameraPos];
desiredResolution = 0.020;
pxPitch = 0.0022;

focalLen = params.CameraParameters1.FocalLength(1) * pxPitch;
WD = focalLen * desiredResolution/pxPitch;
N = 8;

simDataDir = sprintf('%s/new_simulation', stereoDataDir);
listing = dir(simDataDir);

trackDir = sprintf('%s/new_tracks', stereoDataDir);
pts3D = [];
allErr3D = [];
nParticles = 0;
nParticlesSim = 0;
nMissed2D1 = 0;
n3D = 0;
allTraceDiff = [];
for subPathIdx=1:length(listing)
    %% Read relevant 'diverse' dataset only
    dirItem = listing(subPathIdx);
    targetFile = startsWith(dirItem.name, "d_data_1_2");
    if ~(dirItem.isdir && targetFile)
        continue;
    end
    tic
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
    computedTrackPath = sprintf('%s/tracksComputed_%d_%s_border_%d.mat', ...
        curTrackDir, costOfNonAssignment, detectionMode, boundBuffer);
    % Check if PTV result already exists
    if ~isfile(computedTrackPath)
        runPTV(params, nFrames, curSimImgDir, computedTrackPath, ...
            costOfNonAssignment, detectionMode, boundBuffer, 200, 300);
    end
    
    
    %% Setup for evaluation
    curTrackEvalDir = sprintf("%s/evaluation", curTrackDir);
    if ~exist(curTrackEvalDir, 'dir')
        mkdir(curTrackEvalDir);
    end
    load(simTracksPath, 'tracks');
    sL = tracks.tracksL;
    sR = tracks.tracksR;
    sL = sortrows(sL, 'id');
    sR = sortrows(sR, 'id');
    load(computedTrackPath, 'tracks');
    tL = tracks.tracksL;
    tR = tracks.tracksR;
    tL = sortrows(tL, 'id');
    tR = sortrows(tR, 'id');

    %% 1. Match PTV tracks to simulation
    img1Path = sprintf('%s/cam_1_1.tif', curSimImgDir);
    img2Path = sprintf('%s/cam_2_1.tif', curSimImgDir);
    img1 = imread(img1Path, 'TIFF');
    img2 = imread(img2Path, 'TIFF');
%     figTrace = drawTraceFigure(tL, img1, 'computed 1');
%     figTraceSim = drawTraceFigure(sL(22, :), img1, 'simulated 1');
    [trackMatch1, matchIdx1] = matchTracksPTVtoSim(tL, sL, nFrames);
    % Remove matched tracks to visualize the missing ones
%     sLMissed = sL(trackMatch1 == 0, :);
%     if ~isempty(sLMissed)
%         figMissingSim = drawTraceFigure(sLMissed, img1, 'missed simulation');
%     end
    [trackMatch2, matchIdx2] = matchTracksPTVtoSim(tR, sR, nFrames);
    
    %% 2. track analysis for sinking velocity, size distn and carbon flux
    analysis = analyzeTrack(curSimConfig, tL);
%     visualizeAnalysis(analysis);
    
    %% 3. comparison against ground truth of simulation
    analysis.nSimParticlesL = height(sL);
    analysis.nSimParticlesR = height(sR);
    analysis.nDetParticlesL = height(tL);
    analysis.nDetParticlesR = height(tR);
%     
    % a) No. of missed 2D detection as a function of depth
    %       Define depth at which we are able to detect all particle within
    dist = zeros(height(sL), 4);
    traceDiff = zeros(height(sL), 2);
    for sIdx = 1:height(sL)
        tIdx = trackMatch1(sIdx);
        dist(sIdx, 1) = norm(sL.trace{sIdx}.detectedTrace3D(2, 3) - camPoses(1, 3));
        dist(sIdx, 2) = sL.esd(sIdx);
        dist(sIdx, 3) = norm(mean(diff(sL.trace{sIdx}.detectedTrace3D)));
        dist(sIdx, 4) = tIdx > 0;
        if tIdx ~= 0
            dt = tL.trace{tIdx}.detectedTrace2D(matchIdx1(sIdx, 1):matchIdx1(sIdx, 2), :);
            st = sL.trace{sIdx}.detectedTrace2D(matchIdx1(sIdx, 3):matchIdx1(sIdx, 4), :);
            traceDiff(sIdx, :) = mean(abs(dt - st));
            if norm(traceDiff(sIdx, :)) > 15
%                 v = VideoWriter(sprintf("%s/trace_%d.avi", curTrackEvalDir, sIdx));
%                 v.FrameRate = 4;
%                 open(v);
%                 fIdx = matchIdx1(sIdx, 5);
% 
% %                figure;
% %                hold on;
% %                 imshow(img1, [0 255]);
%                 lenMatch = matchIdx1(sIdx, 2) - matchIdx1(sIdx, 1);
%                 for l=1:lenMatch
%                     img1Path = sprintf('%s/cam_1_%d.tif', curSimImgDir,fIdx);
%                     img1 = imread(img1Path, 'TIFF');
%                     d = round(tL.ax_mj(tIdx));
%                     txy = dt(l, :) - d/2;
%                     img1 = insertShape(img1, 'Rectangle', [txy d d], ...
%                         'Color','red', 'LineWidth', 2);
%                     sxy = st(l, :) - d/2;
%                     img1 = insertShape(img1, 'Rectangle', [sxy d d], ...
%                         'Color','yellow', 'LineWidth', 2);
%                     writeVideo(v, img1);
% %                     line([dt(l, 1); st(l, 1)], [dt(l, 2); st(l, 2)], ...
% %                         'Color', rand(3, 1)', ...
% %                         'LineWidth', 1);
%                     fIdx = fIdx + 1;
%                 end
%                 close(v);
%                 line(st(:, 1), st(:, 2), ...
%                     'Color', sL.colour(sIdx, :)./255, ...
%                     'LineWidth', 1);
%                 hold off;
%                 close all; 
%   
                continue;
            end
                
            allTraceDiff = [allTraceDiff; traceDiff(sIdx, :)];
            
        end
    end
    nParticles = nParticles + height(tL);
    depthFilter = abs(dist(:, 1)-250) <= 35;
    mean(dist(sIdx, 1))
    length(find(trackMatch1(depthFilter) == 0))
    nParticlesSim = nParticlesSim + height(sL(depthFilter, :));
    nMissed2D1 = nMissed2D1 + length(find(~trackMatch1));
    dist(dist == 0) = [];
    max(dist)
    min(dist)
    % b) No. of missed 3D tracks
    %       Define volume (i.e. depth) in which the tracks are found 
    n3D = n3D + length(find(analysis.has3D));
    curErr3D = zeros(height(tL), 3);
    err3D = [];
    for tIdx = 1:height(tL)
        if analysis.has3D(tIdx)
            sIdx = find(trackMatch1==tIdx);
            tL3D = tL.trace{tIdx}.detectedTrace3D(matchIdx1(sIdx, 1):matchIdx1(sIdx, 2), :);
            computeESD(focalLen, N, WD, tL.ax_mj(tIdx), pxPitch, tL3D, sL.esd(sIdx));
            sL3D = sL.trace{sIdx}.detectedTrace3D(matchIdx1(sIdx, 3)+1:matchIdx1(sIdx, 4)+1, :);
            invalidFilter = sum(tL3D == 0, 2) == 3;
            tLErr3D = abs(tL3D - sL3D);
            
            tL3D(invalidFilter, :) = [];
            tLErr3D(invalidFilter, :) = [];
            curErr3D(tIdx, :) = mean(tLErr3D);
            pts3D = [pts3D; tL3D];
            err3D = [err3D; tLErr3D];
        end
    end
    allErr3D = [allErr3D; err3D];
    curErr3D(sum(curErr3D == 0) == 3, :) = [];
    
    % c) Error of 3D tracks 

    % d) 
    toc
end

diffPts = max(pts3D) - min(pts3D);
observedVol = diffPts(1) * diffPts(2) * diffPts(3) / 1000;
observedArea = diffPts(1) * diffPts(3);
