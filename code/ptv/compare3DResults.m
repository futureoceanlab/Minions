function [count3D, err3D, falsePos] = compare3DResults(tracks, simTracks, trackMatch, matchIdx)
    % COMPARE 3D RESULTS find the 3D error of matched particles in 
    % simulation and computation
    count3D = 0;
    err3D = [];
    falsePos = 0;
    for tIdx = 1:height(tracks)
        if tracks.has3D(tIdx)
            sIdx = find(trackMatch==tIdx);
            if isempty(sIdx) || simTracks.totalVisibleCount(sIdx) < 5
                continue;
            end
            tracks3D = tracks.trace{tIdx}.detectedTrace3D(matchIdx(sIdx, 1):matchIdx(sIdx, 2), :);
            simTracks3D = simTracks.trace{sIdx}.detectedTrace3D(matchIdx(sIdx, 3):matchIdx(sIdx, 4), :);
            invalidFilter = sum(tracks3D == 0, 2) == 3;
            % Find the difference
            tracksErr3D = abs(tracks3D - simTracks3D);
            tracks3D(invalidFilter, :) = [];
            tracksErr3D(invalidFilter, :) = [];

            if (norm(std(diff(tracks3D))) > 1)
                continue;
            end
            
            if mean(vecnorm(tracksErr3D')) > 0.5
                falsePos = falsePos + 1;
                continue;
            end
            
            count3D = count3D + 1;
            err3D = [err3D; tracksErr3D];
            
        end
    end

%     curErr3D(sum(curErr3D == 0) == 3, :) = [];
end