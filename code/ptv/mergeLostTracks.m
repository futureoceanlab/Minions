function tracks = mergeLostTracks(tracks, lostTracks)
    tracks(tracks.merged == true, :) = [];
    tracks = [tracks; lostTracks];
%     lostFilter = tracks.lost == true;
%     lostIdx = find(lostFilter);
% %     if (size(lostIdx, 1) > 0)
%         lastCentroidIdx = tracks.age(lostFilter) - tracks.consecutiveInvisibleCount(lostFilter);
%         for l=1:size(lostIdx, 1)
%             dT = tracks.trace{lostIdx(l)}.detectedTrace2D;
%             tracks.centroid(lostIdx(l), :) = dT(lastCentroidIdx(l), :);
%         end
%     end
end