function [tracks, lostTracks, nextTrackId] = updateTracksPTV(tracks, particles, ...
    costOfNonAssignment, maxTracks, nextTrackId, lostTracks)
    
    %% Predict using Kalman filter
        for j=1:height(tracks)
            predictedCentroid = predict(tracks.kalmanFilter{j});
            tracks.trace{j} = tracks.trace{j}.addPredicted(predictedCentroid);
            tracks.trace{j} = tracks.trace{j}.addDetected2D(predictedCentroid);
            tracks.trace{j} = tracks.trace{j}.addDetected3D([0 0 0]);
            tracks.centroid(j, :) = predictedCentroid;
        end

    %% Assign detection to current tracks   
        nTracks = height(tracks);
        nDetections = height(particles);
        cost = zeros(nTracks, nDetections);
%         areaRatio = ones(nTracks, nDetections);
    %     axMjRatio = zeros(size(areaRatio));
        for j=1:nTracks
            cost(j, :) = distance(tracks.kalmanFilter{j}, particles.centroid);
%             areaRatio(j, :) = particles.area ./ tracks.area(j);
    %         axMjRatio(j, :) = particles.ecc ./ tracks.ecc(j);
        end
        c = particles.centroid;
%         inImageFilter = c(:, 1) < 50 |c(:, 2) < 50 | c(:, 1) > 2542 | c(:, 2) > 1894;
%         areaRatio(:, inImageFilter) = 1; 
    %     axMjRatio(:, inImageFilter) = 1;

    %     areaRatio(:, isInImage(particles.centroid', 1:nDetections, 100, 2492, 100, 1844)) = ones(nTracks, 1);
    %     cost = cost./max(max(cost));
    %     for j=1:nTracks
    %         cost(j, :) = cost(j, :) + (repmat(tracks.area{j}, [1 nDetections])/particles.area);
        cost(cost > costOfNonAssignment) = Inf;
%         cost(abs(areaRatio - 1) > 0.15) = Inf;
    %     cost(abs(axMjRatio - 1) > 0.5) = Inf;
        idx = munkres(cost);
        assignments = [1:length(idx); idx]';
        I = assignments(:, 2) == 0;
        assignments(I, :) = [];
        unassignedTracks = find(idx == 0);
        unassignedDetections = setdiff(1:nDetections, assignments(:, 2));

    %% update Kalman filter with assigned particles
        numAssignedTracks = size(assignments, 1);
        for j=1:numAssignedTracks
            % In assignments the 1st column represents the track index 
            % and the 2nd column represents the detection index
            rowId = assignments(j, 1);
            detectionIdx = assignments(j, 2);
            centroid = particles.centroid(detectionIdx, :);

            % Correct the estimate of the object's location using the new detection.
            % The method overwrites the internal state and covariance of the
            % Kalman filter object with the corrected measurement (centroid)
            trackedLocation = correct(tracks.kalmanFilter{rowId}, centroid);

            %% Particle data
            % Replace the predicted centroid and bounding box with the detection
    %         tracks.trace{rowId} = tracks.trace{rowId}.updatePredicted(trackedLocation);
%             tracks.trace{rowId} = tracks.trace{rowId}.updateDetected2D(centroid);
            tracks.centroid(rowId, :) = centroid;
            tracks.bbox(rowId, :) = particles.bbox(detectionIdx, :);
            tracks.area(rowId, :) = particles.area(detectionIdx, :);

            tracks.ax_mj(rowId, :) = particles.ax_mj(detectionIdx, :);
            tracks.ax_mn(rowId, :) = particles.ax_mn(detectionIdx, :);
            tracks.or(rowId, :) = particles.or(detectionIdx, :);
            tracks.ecc(rowId, :) = particles.ecc(detectionIdx, :);
            tracks.r2(rowId, :) = particles.r2(detectionIdx, :);
            tracks.per(rowId, :) = particles.per(detectionIdx, :);

            % this is needed to link the track ID to the particle ID currently
            % detected in the frame
            tracks.currentLeftparticleId(rowId) = ...
                particles.id(detectionIdx);

            % set it to 0 here, because the particle position has been replaced
            % above with the real one
            tracks.estimated(rowId) = 0;

            % Update visibility counter. This is updated only if the track is 
            % assigned
            tracks.totalVisibleCount(rowId) = ...
                tracks.totalVisibleCount(rowId) + 1;

            % Reset counter if the track was invisible before
            tracks.consecutiveInvisibleCount(rowId) = 0;
        end   
    % Create new tracks of particles not assigned (appeared or false alarm)
        centroids = particles.centroid(unassignedDetections, :);
        bboxes = particles.bbox(unassignedDetections, :);
        areas = particles.area(unassignedDetections);
        ids = particles.id(unassignedDetections);

        ax_mjs = particles.ax_mj(unassignedDetections);
        ax_mns = particles.ax_mn(unassignedDetections);
        ors = particles.or(unassignedDetections);
        eccs = particles.ecc(unassignedDetections);
        r2s = particles.r2(unassignedDetections);
        pers = particles.per(unassignedDetections);

        newTrackCount = 0;
        for j=1:size(centroids, 1)
            % when system saturates stop adding new tracks
            if(height(tracks) >= maxTracks)
                continue;
            end

            centroid = centroids(j, :);
            bbox = bboxes(j, :);
            area = double(areas(j, :));

            % Create a Kalman filter object for new detections only.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', centroid, [50 20], [20, 5], 10);
            trace = particleTrace;
            trace = trace.addDetected2D(centroid);
    %         trace = trace.addPredicted(centroid);

            % Create a new track
            newTrack = table(nextTrackId, {kalmanFilter}, 0, 1, 0, 0, ...
                 255*rand(3, 1)', ids(j), centroid, bbox, area, [NaN NaN], ...
                 pers(j), ax_mjs(j), ax_mns(j), ors(j), eccs(j), r2s(j), ...
                 [NaN NaN NaN], [NaN NaN], 0, false, false,{trace}, ...
                'VariableNames', {'id', 'kalmanFilter', 'age',  'totalVisibleCount', ...
                'totalInvisibleCount', 'consecutiveInvisibleCount', 'colour', ...
                'currentLeftparticleId', 'centroid', 'bbox', 'area', 'rightCoord2D', ...
                'per', 'ax_mj', 'ax_mn', 'or', 'ecc', 'r2', ...
                'worldCoordinates', 'length', 'estimated', 'lost', ...
                'merged', 'trace'
            });

            % Add it to the array of system.tracks.
            tracks = [tracks; newTrack];

            % Increment the next id
            nextTrackId = nextTrackId + 1;
            newTrackCount = newTrackCount + 1;
        end
    % Update unassigned tracks 
        row = unassignedTracks;

        tracks.totalInvisibleCount(row) = ...
            tracks.totalInvisibleCount(row) + 1;

        tracks.consecutiveInvisibleCount(row) = ...
            tracks.consecutiveInvisibleCount(row) + 1;

        newInvisibleTracks = tracks(tracks.consecutiveInvisibleCount == 1, :);
        if height(newInvisibleTracks) >= 2
            pDist = zeros(height(newInvisibleTracks), height(particles));
            for j=1:height(newInvisibleTracks)
                pDist(j, :) = vecnorm(particles.centroid - newInvisibleTracks.centroid(j, :), 2, 2);
            end

            [~, closestIdx] = min(pDist,[], 2);
            [C, ia, ic] = unique(closestIdx);
            % Note the particle who is a merged outcome of two particles
            % as 'merged' to be later removed
            if length(C) ~= length(closestIdx)
    %             drawTrace(img1, newInvisibleTracks);
                dupIdx = setdiff(1:length(closestIdx), ia);
                dupVal = closestIdx(dupIdx);
                dupFilter = ismember(tracks.currentLeftparticleId, dupVal) ...
                    & tracks.age == 0;
                tracks.merged(dupFilter) = true;
    %             close all;
            end
        end
    % Clean lost tracks
    c = tracks.centroid;
    minX = 1; maxX = 2592;
    minY = 1; maxY = 1944;
    lostThreshold = 5;
    outImageFilter = (c(:, 1) < minX | c(:, 1) > maxX ...
               | c(:, 2) < minY | c(:, 2) > maxY);
    lostFilter = tracks.consecutiveInvisibleCount == lostThreshold;
    lostFilter = outImageFilter & lostFilter;
    if sum(lostFilter) > 0
        tracks.lost(lostFilter) = true;
        lostTracks = [lostTracks; tracks(lostFilter, :)];
        tracks(lostFilter, :) = [];
    end
end