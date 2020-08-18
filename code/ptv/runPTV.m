function runPTV(params, nFrames, simImgDir, computedTrackPath, ...
            costOfNonAssignment, detectionMode, boundBuffer, ...
            minDepth, maxDepth, visibilityRatio)
        
    % RUN PTV   PTV execution software. Run PTV, match
    %           3D correspondence and update the tracks 
    %% Parse the camera parameters
    cameraMatrix1 = squeeze(params.camMatrix(1, :, :));
    cameraMatrix2 = squeeze(params.camMatrix(2, :, :));
    intrinsics = params.CameraParameters1.IntrinsicMatrix';

    %% vision setting
    % blob Analyzer
    tracksL = table();
    tracksR = table();
    lostTracksL = table();
    lostTracksR = table();
    maxTracks = 400;
    nextTrackIdL= 1;
    nextTrackIdR = 1;

    %% Main Loop
    for i = 1:nFrames        
        img1Path = sprintf('%s/cam_1_%d.tif', simImgDir, i);
        img2Path = sprintf('%s/cam_2_%d.tif', simImgDir, i);
        img1 = imread(img1Path, 'tiff');
        img2 = imread(img2Path, 'tiff');
        img1 = double(img1); 
        img2 = double(img2);
        img1 = imgaussfilt(img1, 2);
        img2 = imgaussfilt(img2, 2); 
        img1 = rescale(log2(img1));
        img2 = rescale(log2(img2));
        %% 1. Detect partices on left and right
        [particles1, particles2] = detectParticles(img1, img2, detectionMode, boundBuffer);
    
        %% 2. Update tracks (Run PTV)
        [tracksL, lostTracksL, nextTrackIdL] = updateTracksPTV(tracksL, particles1, costOfNonAssignment, maxTracks, nextTrackIdL, lostTracksL, visibilityRatio);
        [tracksR, lostTracksR, nextTrackIdR] = updateTracksPTV(tracksR, particles2, costOfNonAssignment, maxTracks, nextTrackIdR, lostTracksR, visibilityRatio);

        %% 3. Triangulate particles for assigned and new tracks
        % try matching the particles in the current assigned tracks. Unassigned
        % tracks have NaN as 'currentLeftparticleIds'.
        rowsToUpdate = ~isnan(tracksL.currentLeftparticleId);
        hasNewAssignment = ~isnan(tracksR.currentLeftparticleId);
        pR = tracksR.centroid(hasNewAssignment, :);
        for tIdx=1:height(tracksL)
            if rowsToUpdate(tIdx)
                lPts = tracksL.centroid(tIdx, :);
                
                % project left centroid point onto 3D space coordinate
                % along the epipolar line. In particular, we look at the 
                % closest and farthest possible 3D points
                pt3dLeft = (minDepth.*inv(intrinsics) * [lPts(1); lPts(2); 1])';
                pt3dRight = (maxDepth.*inv(intrinsics) * [lPts(1); lPts(2); 1])';
                
                % project those 3D points to right image coordinate system
                points = projectPoints([pt3dLeft; pt3dRight], cameraMatrix2')';
                
                % Epipolar search area
                minY = max([0, min([points(1, 2), points(2, 2)])-5]);
                maxY = min([1944, max([points(1, 2), points(2, 2)])+5]);
                minX = max([0, min([points(1, 1), points(2, 1)])-5]);
                maxX = min([2592, max([points(1, 1), points(2, 1)])+5]);
                if minY > 1944 || maxY < 0 || minX > 2592 || maxX < 0
                    continue;
                end
                % Find particles that fall in this area
                xyFilter = pR(:, 1) > minX & pR(:, 1) < maxX ...
                    & pR(:, 2) > minY & pR(:, 2) < maxY;
                rightFilteredXY = pR(xyFilter, :);
                if isempty(rightFilteredXY)
                    % None found, continue
                    continue;
                end
                
                % We now search for the closest point to the epipolar line
                v1 = [points(1, :), 0];
                v2 = [points(2, :), 0];
                pt = [rightFilteredXY, zeros(size(rightFilteredXY, 1), 1)];
                v1_ = repmat(v1, size(pt, 1), 1);
                v2_ = repmat(v2, size(pt, 1), 1);
                a = v1_ - v2_;
                b = pt - v2_;
                % point to epipolar line distance
                ptToLineDist = sqrt(sum(cross(a, b, 2).^2, 2))./sqrt(sum(a.^2, 2));
                [~, minI] = min(ptToLineDist);
                % Trianglate the 3D point and update the tracks
                [point3d, ~] = triangulate(lPts, rightFilteredXY(minI, :), cameraMatrix1, cameraMatrix2);
                tracksL.trace{tIdx} = tracksL.trace{tIdx}.updateDetected3D(point3d);
                if tracksL.has3D(tIdx) == false
                    % Mark particles whose 3D counterpart was found
                    tracksL.has3D(tIdx) = true;
                end
            end
        end
        % Update age of all the tracks
        if ~isempty(tracksL)
            tracksL.age = tracksL.age + 1;
        end
        if ~isempty(tracksR)
            tracksR.age = tracksR.age + 1;
        end
        if ~isempty(lostTracksL)
            lostTracksL.age = lostTracksL.age + 1;
        end
        if ~isempty(lostTracksR)
            lostTracksR.age = lostTracksR.age + 1;
        end
    end
    % all frames have been analyzed. Merge the lost tracks with the current
    % tracks
    [tracksL, fL]= mergeLostTracks(tracksL, lostTracksL, visibilityRatio);
    [tracksR, fR] = mergeLostTracks(tracksR, lostTracksR, visibilityRatio);
    tracks = struct('tracksL', tracksL, 'tracksR', tracksR, 'fL', fL, 'tR', fR);

    save(computedTrackPath, 'tracks');
end