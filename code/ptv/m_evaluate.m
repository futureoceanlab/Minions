clear;
%% Simulation parameters configurations
simMode = "monotonic";

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
simDataDir = sprintf('%s/simulation', stereoDataDir);
listing = dir(simDataDir);