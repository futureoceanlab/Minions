function [traceMatch, matchIdx] = matchTracksPTVtoSim(tracks, tracksSim, nFrames)

%         figTrace = drawTraceFigure(tracks, img, 'computed');
%         curTracePath = sprintf("%s/trace_%d_%s_border_%d_%d.fig", ...
%             curTrackEvalDir, costOfNonAssignment, detectionMode, ...
%             boundBuffer, camIdx);
%         curTraceImgPath = sprintf("%s/trace_%d_%s_border_%d.png", ...
%             curTrackEvalDir, costOfNonAssignment, detectionMode, ...
%             boundBuffer);
% %         saveas(figTrace, curTraceImgPawth, 'png');
%         savefig(curTracePath);
% 
%         figTraceSim = drawTraceFigure(tracksSim, img, 'simulated');
%         curSimTracePath = sprintf("%s/trace_%d_%s_sim_%d.fig", ...
%             curTrackEvalDir, costOfNonAssignment, detectionMode, ...
%             camIdx);
%         curSimTraceImgPath = sprintf("%s/trace_%d_%s_sim.png", ...
%             curTrackEvalDir, costOfNonAssignment, detectionMode);
%         savefig(curSimTracePath);
% %         saveas(figTraceSim, curSimTraceImgPath, 'png');
% 
%         close all;
        % [globalStartIdx globalEndIdx]
        
        % Match between tracked particles with simulated particles
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
    % Remove matched tracks to visualize the missing ones
%         tracksSim(~(traceMatch == 0), :) = [];
%         traceMatch(traceMatch == 0) = [];
%         tracks(traceMatch, :) = [];
%         if ~isempty(tracks)
%             figMissingTrace = drawTraceFigure(tracks, img, 'missed computed');
%         end
%         if ~isempty(tracksSim)
%             figMissingSim = drawTraceFigure(tracksSim, img, 'missed simulation');
%         end
end