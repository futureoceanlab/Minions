function [count2D, err2D, trackMatch, missedSim] = compare2DResults(track, simTrack, trackMatch, matchIdx, camPoses)
    % COMPARE 2D RESULTS match computed result to simulation in 2D real
    
    dist = zeros(height(simTrack), 4);
    err2D = [];
    count2D = 0;
    missedSim = table;
    
    for sIdx = 1:height(simTrack)
        tIdx = trackMatch(sIdx);
        % actual depth from the camera center (left)
        dist(sIdx, 1) = norm(simTrack.trace{sIdx}.detectedTrace3D(2, 3) - camPoses(3));
        % ESD
        dist(sIdx, 2) = simTrack.esd(sIdx);
        % velocity in 3D
        dist(sIdx, 3) = norm(mean(diff(simTrack.trace{sIdx}.detectedTrace3D)));
        if tIdx ~= 0
            dt = track.trace{tIdx}.detectedTrace2D(matchIdx(sIdx, 1):matchIdx(sIdx, 2), :);
            st = simTrack.trace{sIdx}.detectedTrace2D(matchIdx(sIdx, 3):matchIdx(sIdx, 4), :);
            traceDiff = mean(abs(dt - st));
            if (~isempty(find(isnan(traceDiff), 1)))
                trackMatch(sIdx) = 0;
                missedSim = [missedSim; simTrack(sIdx, :)];
               continue; 
            end
            % A particle found in our PTV
            dist(sIdx, 4) = 1;
            count2D = count2D + 1;
            err2D = [ err2D; abs(dt-st)];
        else
           missedSim = [missedSim; simTrack(sIdx, :)];
        end
    end
end