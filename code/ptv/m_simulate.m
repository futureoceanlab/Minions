clear;
%% Simulation parameters configurations
simMode = "monotonic";
shapes = ["sphere"; "cylinder"];
sRadius = [25; 50; 100; 200; 400]; % minRadius to maxRadius (um)
cRadius = [25; 50; 100; 200; 400];
cHeights = [25; 50; 100; 200; 400];
concentration = [250; 500; 1000; 2000; 3000]; % particles / m^3
sinkingRate = [25; 50; 100; 200]; % m/day
% unit vectors in x, y, z (each column) direction relative to left camera
fallingDirections = [0          1           0; ...
                    sqrt(1/3)   sqrt(2/3)   0; ...
                     0          sqrt(.5)   sqrt(.5); ...
                     sqrt(1/3)  sqrt(1/3)   sqrt(1/3)]; 
nFrames = 180;


%% Path configuration
stereoDataDir = sprintf('data/stereocamera_%d',1);

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
simDataDir = sprintf('%s/simulationasdasd', stereoDataDir);
if ~exist(simDataDir)
    mkdir(simDataDir);
end

rndSeed = 0;
%% Monotonic Simulator 
for sIdx=1:length(shapes)
curShape = shapes(sIdx);

for rIdx=1:length(sRadius)
curSRadius = sRadius(rIdx);
curCRadius = cRadius(rIdx);
curCHeights = cHeights(rIdx);

for cIdx=1:length(concentration)
curConcentration = concentration(cIdx) * curSRadius * 2;

for srIdx=1:length(sinkingRate)
curSinkingRate = sinkingRate(srIdx);

for fdIdx=1:length(fallingDirections)
curDirection = fallingDirections(fdIdx, :);

%% Simulation
% Check/create simulation folder
curSimDirName = sprintf('m_data_%s_%d_%d_%d_%d',...
                        curShape, curSRadius, curConcentration, ...
                        curSinkingRate, fdIdx);
curSimDir = sprintf('%s/%s', simDataDir, curSimDirName);
if exist(curSimDir, 'dir')
    fprintf('directory: %s already exists! Skipping...\n', simDataDir);
%     continue;
else
    mkdir(curSimDir);
    fprintf('%s\n', curSimDir);
end

if curShape == "sphere"
    sphereRatio = 1;
else
    sphereRatio = 0;
end

% Save the configuration
curSimConfig = struct('sphereRatio', sphereRatio, ...
    'sRadius', curSRadius, ...
    'cRadius', curCRadius, ...
    'cHeights', curCHeights, ...
    'concentration', curConcentration, ...
    'sinkRate', curSinkingRate, ...
    'direction', curDirection, ...
    'nFrames', nFrames);

save(sprintf('%s/config.mat', curSimDir), 'curSimConfig');

% Run simulator
simulator(params, curSimDir, curSimConfig, simMode, rndSeed);
rndSeed = rndSeed + 1;
end
end
end
end         
end
