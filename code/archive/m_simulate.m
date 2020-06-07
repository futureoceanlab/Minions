clear;

%% Simulation parameters configurations
shapes = ["sphere"; "cylinder"];
particleRadius = [25; 50; 100; 200; 400]; % minRadius to maxRadius (um)
concentration = [100; 500; 1000; 2000;]; % particles / m^3
sinkingRate = [25; 50; 100; 200;]; % m/day
% unit vectors in x, y, z (each column) direction relative to left camera
fallingDirections = [0          1           0; ...
                    sqrt(1/3)   sqrt(2/3)   0; ...
                     0          sqrt(.5)   sqrt(.5); ...
                     sqrt(1/3)  sqrt(1/3)   sqrt(1/3)]; 
nFrames = 180;

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
simDataDir = sprintf('%s/simulation', stereoDataDir);
if ~exist(simDataDir)
    mkdir(simDataDir);
end
%% Monotonic Simulator 
for sIdx=1:length(shapes)
curShape = shapes(sIdx);

for rIdx=1:length(particleRadius)
curRadius = repmat(particleRadius(rIdx), [1 2]);

for cIdx=1:length(concentration)
curConcentration = concentration(cIdx);

for srIdx=1:length(sinkingRate)
curSinkingRate = sinkingRate(srIdx);

for fdIdx=1:length(fallingDirections)
curDirection = fallingDirections(fdIdx, :);

% Check/create simulation folder
curSimDirName = sprintf('m_data_%s_%d_%d_%d_%d',...
                        curShape, curRadius, curConcentration, ...
                        curSinkingRate, fdIdx);
curSimDir = sprintf('%s/%s', simDataDir, curSimDirName);
if exist(curSimDir, 'dir')
    fprintf('directory: %s already exists! Skipping...\n', simDataDir);
    continue;
else
    mkdir(curSimDir);
    fprintf('%s\n', curSimDir);
end

% Save the configuration
curSimConfig = struct('sphereRatio', curShape=="sphere", ...
    'radius', curRadius, ...
    'concentration', curConcentration, ...
    'sinkingRate', curSinkingRate, ...
    'direction', curDirection, ...
    'frames', nFrames);

save(sprintf('%s/config.mat', curSimDir), 'curSimConfig');

% Run simulator
ptvSimulator = MinionPTV.simulator(stereoParamPath, curSimConfig, curSimDir);
ptvSimulator.run();

end
end
end
end         
end
