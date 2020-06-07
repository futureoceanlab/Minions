clear;
%% Simulation parameters configurations
simMode = "diverse";
nData = 3;
sRadiusRange = [25 120; 120 195; 195 320; 320 520; 520 850; 850 1400]; % minRadius to maxRadius (um)
cRadiusRange = [25 120; 120 195; 195 320; 320 520; 520 850; 850 1400];
cHeightsRange = [25 120; 120 195; 195 320; 320 520; 520 850; 850 1400];
concentrationRange = [80 300; 20 40; 8 18; 3 13; 0.5 1.5; 0.25 1.25]; % No. / m^3 / um
sinkRateRange = [50 300; 30 200; 20 135; 30 400; 50 400; 50 400]; % particles / m^3
sinkRateMean = mean(sinkRateRange, 2);
sinkRateSTD = abs(sinkRateRange(:, 1) - sinkRateMean);
sinkRateDetail = [sinkRateMean sinkRateSTD];
% unit vectors in x, y, z (each column) direction relative to left camera
fallingDirections = [0          1           0; ...
                     0          sqrt(.5)   sqrt(.5); ...
                     sqrt(1/3)  sqrt(1/3)   sqrt(1/3)]; 
nFrames = 1800;


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
simDataDir = sprintf('%s/new_simulation', stereoDataDir);
if ~exist(simDataDir)
    mkdir(simDataDir);
end
rndSeed = 0;

%% Monoton  ic Simulator 
for sIdx=1:nData
sphereRatio = rand();
curConcentration = diff(concentrationRange, [], 2).*rand([size(concentrationRange, 1) 1]) + concentrationRange(:, 1);
curConcentration = (2 * curConcentration).*mean(sRadiusRange, 2);

for fdIdx=2:length(fallingDirections)
curDirection = fallingDirections(fdIdx, :);
%% Simulation
% Check/create simulation folder
curSimDirName = sprintf('d_data_%d_%d', sIdx, fdIdx);
curSimDir = sprintf('%s/%s', simDataDir, curSimDirName);
if exist(curSimDir, 'dir')
    fprintf('directory: %s already exists! Skipping...\n', simDataDir);
%     continue;
else
    mkdir(curSimDir);
    fprintf('%s\n', curSimDir);
end

% Save the configuration
curSimConfig = struct('sphereRatio', sphereRatio, ...
    'sRadiusRange', sRadiusRange, ...
    'cRadiusRange', cRadiusRange, ...
    'cHeightsRange', cHeightsRange, ...
    'concentration', curConcentration, ...
    'sinkRateDetail', sinkRateDetail, ...
    'direction', curDirection, ...
    'nFrames', nFrames);

save(sprintf('%s/config.mat', curSimDir), 'curSimConfig');

% Run simulator
simulator(params, curSimDir, curSimConfig, simMode, rndSeed);
rndSeed = rndSeed + 1;
rng(rndSeed,'twister');

end
end