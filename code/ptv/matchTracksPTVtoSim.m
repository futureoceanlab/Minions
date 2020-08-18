function [traceMatch, matchIdx] = matchTracksPTVtoSim(tracks, tracksSim, nFrames)
    % MATCH TRACKS PTV TO SIM find the simulation particle corresponding
    % to the particle using the PTV
    
    % First, convert age to global time (index)
    globalTrackerT = indexTraces(tracks, nFrames);
    globalTrackerS = indexTraces(tracksSim, nFrames);
    traceDiff = zeros(height(tracks), 2);
    traceMatch = zeros(height(tracksSim), 1);
    matchIdx = zeros(height(tracksSim), 5);
    missed = 0;
    for i = 1:height(tracksSim)
        st = tracksSim.trace{i}.detectedTrace2D;
        globalIdxS = repmat(globalTrackerS(i, :), [height(tracks), 1]);
        % choose whichever global start is later
        maxStartIdx = max(globalTrackerT(:, 1), globalIdxS(:, 1));
        % Choose whichever global end is earler
        minEndIdx = min(globalTrackerT(:, 2), globalIdxS(:, 2));

        % start index of detectedTrace for "tracks"
        tStartIdx = abs(globalTrackerT(:, 1) - maxStartIdx) + 1;
        % start index of detecetedTrace for "tracksSim"
        sStartIdx = abs(globalIdxS(:, 1) - maxStartIdx) + 1;
        % length of detectedTrace
        endIdx = minEndIdx - maxStartIdx + 1;
        for j=1:height(tracks)
            dt = tracks.trace{j}.detectedTrace2D(tStartIdx(j):tStartIdx(j)+endIdx(j)-1, :);
            traceDiff(j, :) = mean(abs(dt - st(sStartIdx(j):sStartIdx(j)+endIdx(j)-1, :)));
        end
        d = vecnorm(traceDiff, 2, 2);
        % Find the simulation particle whose trace is the closest 
        % to that of current particle
        [minD, minIdx] = min(d);
        % Remove matches that are too far
        if (minD > 50)
            missed = missed + 1;
            continue;
        end
        if (minD > 15)
            dt = tracks.trace{minIdx}.detectedTrace2D;
%             display("stop");
        end
        
        matchIdx(i, :) = [  tStartIdx(minIdx), ...
                            tStartIdx(minIdx) + endIdx(minIdx) - 1,...
                            sStartIdx(minIdx),...
                            sStartIdx(minIdx) + endIdx(minIdx) - 1, ...
                            maxStartIdx(minIdx)];
        traceMatch(i) = minIdx;
    end
end