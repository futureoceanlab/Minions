function globalTracker = indexTraces(tracks, nFrames)
    % INDEX TRACES Conmpute the start and end of the presence of the
    % particle based on the age
    globalTracker = zeros(height(tracks), 2);
    for i=1:height(tracks)
        gStartIdx = nFrames - tracks.age(i) + 1;
        dtLen = size(tracks.trace{i}.detectedTrace2D, 1);
        gEndIdx = gStartIdx + dtLen - 1;
        globalTracker(i, :) = [gStartIdx, gEndIdx];
    end
end