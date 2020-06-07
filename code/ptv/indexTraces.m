function globalTracker = indexTraces(tracks, nFrames)
    globalTracker = zeros(height(tracks), 2);
    for i=1:height(tracks)
        gStartIdx = nFrames - tracks.age(i) + 1;
        dtLen = size(tracks.trace{i}.detectedTrace2D, 1);
        gEndIdx = gStartIdx + dtLen - 1;
        globalTracker(i, :) = [gStartIdx, gEndIdx];
    end
end