function [tracks, finalNoiseTracks] = mergeLostTracks(tracks, lostTracks, visibilityRatio)
    tracks = [tracks; lostTracks];

    totalVisibles = tracks.totalVisibleCount;
    totalInvisibles = tracks.totalInvisibleCount;
    ages = tracks.age;
    visibility = totalVisibles./(totalVisibles+totalInvisibles);
    finalNoise = (visibility < visibilityRatio | ages < 25);% & ~has3D;
    finalNoiseTracks = tracks(finalNoise, :);
    tracks(finalNoise, :) = [];
end