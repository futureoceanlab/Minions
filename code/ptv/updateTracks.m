function tracks = updateTracks(tracks, wasInFrame, id, coord2D, coord3D, esd)
    if isempty(tracks) || (sum(tracks.id == id) == 0)
        if wasInFrame
            trace = particleTrace;
            newTrack = table(id, 0, 0, 0, 0, ...
                 255*rand(3, 1)', esd, {trace}, ...
                'VariableNames', {'id', 'age',  'totalVisibleCount', ...
                'totalInvisibleCount', 'consecutiveInvisibleCount', 'colour', ...
                'esd', 'trace'
            });
            tracks = [tracks; newTrack];
        else
            return;
        end
    end
    tIdx = find(tracks.id == id);
    if ~isempty(tIdx)
        if wasInFrame
            tracks.totalVisibleCount(tIdx) = tracks.totalVisibleCount(tIdx) + 1;
            tracks.trace{tIdx} = tracks.trace{tIdx}.addDetected2D(coord2D);
            tracks.trace{tIdx} = tracks.trace{tIdx}.addDetected3D(coord3D);
            tracks.consecutiveInvisibleCount(tIdx) = 0;
        else
            if tracks.age(tIdx) > 0
                tracks.totalInvisibleCount(tIdx) = tracks.totalInvisibleCount(tIdx) + 1;
                tracks.consecutiveInvisibleCount(tIdx) = tracks.consecutiveInvisibleCount(tIdx) + 1;
            end
        end
    end
end