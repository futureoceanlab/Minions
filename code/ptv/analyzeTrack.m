function [analysisResult] = analyzeTrack(config, tracks)
    
    sRadiusRange = config.sRadiusRange ./ 1000;
    sRadiusRange(1, 1) = 0; 
    sRadiusRange(end, end) = inf;
    nBins = size(sRadiusRange, 1);
    concentrationBinCounts = zeros(nBins, 1);
    speedBinCounts = zeros(nBins, 1);
    speeds = zeros(nBins, height(tracks));
    has3D = zeros(height(tracks), 1);
    % Sort by ESD
    for i=1:height(tracks)
        rIdx = find((sRadiusRange(:, 1) < (tracks.ax_mj(i)*0.02)) & (sRadiusRange(:, 2) > (tracks.ax_mj(i)*0.02)));
        concentrationBinCounts(rIdx) = concentrationBinCounts(rIdx) + 1;
        % Compute speed of particles whose 3D points are known
        if size(tracks.trace{i}.detectedTrace3D, 1) > 1
            diffTrace = diff(tracks.trace{i}.detectedTrace3D);
            % remove difference at index at which there was no detection
            diffFilter = sum(tracks.trace{i}.detectedTrace3D == 0, 2) == 3;
            % There needs to be two consecutive 3D coordinates. This is
            % discovered by convolving with [1 1];
            diffFilter = conv(diffFilter, [1 1]);
            diffFilter = diffFilter(2:end-1) > 0;
            diffTrace(diffFilter, :) = [];
            if ~isempty(diffTrace)
                % Compute the norm of average sinking rate in 3 axis
                avgSpeed = norm(mean(diffTrace));
                speedBinCounts(rIdx) = speedBinCounts(rIdx) + 1;
                speeds(rIdx, speedBinCounts(rIdx)) = avgSpeed;
                has3D(i, 1) = 1;
            end
        end
    end
    %% Concentration Calculation
    % Note: binCounts need to be normalized to the volume of water
    % monitored
    observedVolume = 1;
    
    %% Speed Calculation
    % convert mm/s to m/day
    speeds = speeds .* 86.4;
    avgSpeeds = sum(speeds, 2)./speedBinCounts;
    avgSpeeds(isnan(avgSpeeds)) = 0;
    stdSpeeds = zeros(nBins, 1);
    for i=1:nBins
        if speedBinCounts(i) > 0
            stdSpeeds(i) = std(speeds(i, 1:speedBinCounts(i)));
        end
    end
    analysisResult = struct("avgSpeeds", avgSpeeds, ...
                            "stdSpeeds", stdSpeeds, ...
                            "speeds", speeds, ...
                            "observedVolume", observedVolume, ...
                            "nBins", nBins, ...
                            "concentrationBinCounts", concentrationBinCounts, ...
                            "speedBinCounts", speedBinCounts, ...
                            "has3D", has3D);
end