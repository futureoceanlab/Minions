clear;
% close all;

simulation = 1;

% %% Evaluation of PTV
% % run PTV 
% 
% % load all of the data
% load('tracks.mat', 'tracks');
% tracks(tracks.totalVisibleCount < 3, :) = [];
% % size distribution
% % binning with size of particles at every 50um
% maxParticleSize = max(tracks.area); % area in px
% nBin = 10;
% binRes = maxParticleSize/nBin;
% sizeDistn = zeros(nBin, 1);
% trackBinIdx = ceil(tracks.area/binRes);
% for i=1:height(tracks)
%     sizeDistn(trackBinIdx(i)) = sizeDistn(trackBinIdx(i)) + 1;
% end
% bins = (1:nBin)*binRes*(0.02^2); % mm^2
% figure;
% plot(bins', sizeDistn);
% 
% % average speed distribtion as a function of size (scatter plot)
% v = zeros(height(tracks), 1);
% for i = 1:height(tracks)
%     v(i) = norm(mean(diff(tracks.trace{i}.detectedTrace)));
% end
% figure;
% scatter(tracks.area, v);
% 
% % Determination of irregularities as a function size(?)

%% Evaluation for simulation
if simulation

dirPath = 'simulationImg_1_1';
simPath = sprintf('%s/tracksSimulated.mat', dirPath);
comPath = sprintf('%s/tracksComputed.mat', dirPath);
nFrames = 20;
% 1. Display the trace of particles
load(simPath, 'tracks');
tracksSim = tracks;
load(comPath, 'tracks');
diagnosePath = false;
if diagnosePath == true
    traceSim = tracksSim.trace{44}.detectedTrace;
    trace = tracks.trace{4}.detectedTrace;
    bboxW = tracks.bbox(35, 3:4);

    for i=1:nFrames
        close all;
        img1Path = sprintf('bokeh_cam_1_%d.tif', i);
        img1 = imread(img1Path, 'tiff'); 
        bbox = [int32(floor(trace(i, :))) - floor(bboxW/2), bboxW];
        bboxSim = [int32(floor(traceSim(i, :))) - floor(bboxW/2), bboxW];
        figure;
        hold on;
        imshow(img1);
        rectangle('Position', bbox', 'FaceColor', 'none', 'EdgeColor', 'blue');
        rectangle('Position', bboxSim', 'FaceColor', 'none', 'EdgeColor', 'red');

    end
end


nTracks = height(tracksSim);
nMatched = 0;
for i=1:nTracks
    tSim = tracksSim(i, :);
    d = vecnorm(tracks.centroid - tSim.centroid, 2, 2);
    [minDist, minIdx] = min(d);
    if (minDist > 100) 
        continue;
    end
    nMatched = nMatched + 1;
    t = tracks(minIdx, :);
    startIdx = 1;
    if (tSim.age ~= t.age)
        startIdx = tSim.age - t.age + 1;
    end
    endIdx = size(tSim.trace{1}.detectedTrace, 1) - startIdx + 1;
%     startIdx = tSim.age - t.age + 1;
%     tSim.age - t.age
%     tSim.totalVisibleCount - t.totalVisibleCount
    tSim.trace{1}.detectedTrace(startIdx:end, :) - t.trace{1}.detectedTrace(1:endIdx, :)
end

% for i=1:nFrames
%     % 0. load the simulation
%     tableName = sprintf('groundtruth_%d.mat', i);
%     load(tableName, 'frameTable');

% end
end