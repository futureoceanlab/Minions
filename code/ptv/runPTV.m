function runPTV(params, nFrames, simImgDir, computedTrackPath, ...
            costOfNonAssignment, detectionMode, boundBuffer, ...
            minDepth, maxDepth)
    cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
    cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
        params.RotationOfCamera2, params.TranslationOfCamera2);
    F = params.FundamentalMatrix;
    f1 = params.CameraParameters1.FocalLength;
    p1 = params.CameraParameters1.PrincipalPoint;
    pxPitch = 0.0022;
    intrinsics = [f1(1), 0, p1(1);
              0, f1(2), p1(2);
              0, 0, 1]; % mm

    %% vision setting
    % blob Analyzer
    tracksL = table();
    tracksR = table();
    lostTracksL = table();
    lostTracksR = table();
    maxTracks = 400;
    nextTrackIdL= 1;
    nextTrackIdR = 1;
    %% Main loop
    for i = 1:nFrames
        img1Path = sprintf('%s/cam_1_%d.tif', simImgDir, i);
        img2Path = sprintf('%s/cam_2_%d.tif', simImgDir, i);
        img1 = rescale(log2(double(imgaussfilt(imread(img1Path, 'tiff'), 3)))); 
        img2 = rescale(log2(double(imgaussfilt(imread(img2Path, 'tiff'), 3))));
%         img1 = fgDetector1(img1);
%         img2 = fgDetector2(img2);
%         if nFrames < nTrainingFrames
%             continue;
%         end
    %% Detect particles on left and right
%         imshow(img1);
        [particles1, particles2] = detectParticles(img1, img2, detectionMode, boundBuffer);
    %% Update tracks
        [tracksL, lostTracksL, nextTrackIdL] = updateTracksPTV(tracksL, particles1, costOfNonAssignment, maxTracks, nextTrackIdL, lostTracksL);
        [tracksR, lostTracksR, nextTrackIdR] = updateTracksPTV(tracksR, particles2, costOfNonAssignment, maxTracks, nextTrackIdR, lostTracksR);

    %% Triangulate particles for assigned and new tracks
        % reset all coordinate data for the current tracks
        h = height(tracksL);

        % try matching the particles in the current assigned tracks. Unassigned
        % tracks have NaN as 'currentLeftparticleIds'.
        rowsToUpdate = ~isnan(tracksL.currentLeftparticleId);
        % particle IDs of the detections.
        trackedParticlesIds = tracksL.currentLeftparticleId(rowsToUpdate);
        trackFilter = tracksL.id == trackedParticlesIds;
        trackFilterIdx = find(trackFilter);
        % each row number corresponds to a particle IDs of the detection.
%         trackedParticlesData = tracksL(trackFilter, :);
        
        if(~isempty(trackFilterIdx))
            % the rows in trackedParticlesData, trackIds and data match the same
            % left particle IDs because they have the same sort


            for j=1:size(trackFilterIdx, 1)
                tIdx = trackFilterIdx(j);
                lPts = tracksL.centroid(tIdx, :);
%                 lPts = trackedParticlesData.centroid(j, :);
                pt3dLeft = (minDepth.*inv(intrinsics) * [lPts(1); lPts(2); 1])';
                pt3dRight = (maxDepth.*inv(intrinsics) * [lPts(1); lPts(2); 1])';
                points = projectPoints([pt3dLeft; pt3dRight], cameraMatrix2')';

                minY = max([0, min([points(1, 2), points(2, 2)])-10]);
                maxY = min([1944, max([points(1, 2), points(2, 2)])+10]);
                minX = max([0, min([points(1, 1), points(2, 1)])-10]);
                maxX = min([2592, max([points(1, 1), points(2, 1)])+10]);

                xyFilter = particles2.centroid(:, 1) > minX & particles2.centroid(:, 1) < maxX ...
                    & particles2.centroid(:, 2) > minY & particles2.centroid(:, 2) < maxY;
                rightFilteredXY = particles2.centroid(xyFilter, :);
                if isempty(rightFilteredXY)
                    continue;
                end
                v1 = [points(1, :), 0];
                v2 = [points(2, :), 0];
                pt = [rightFilteredXY, zeros(size(rightFilteredXY, 1), 1)];
                v1_ = repmat(v1, size(pt, 1), 1);
                v2_ = repmat(v2, size(pt, 1), 1);
                a = v1_ - v2_;
                b = pt - v2_;
                % point to epipolar line distance
                ptToLineDist = sqrt(sum(cross(a, b, 2).^2, 2))./sqrt(sum(a.^2, 2));
    %             % size ratio
    %             areaRatio = particles2.area(xyFilter, :) ./ trackedParticlesData.area(j, :);
    %             axMjRatio = particles2.ax_mj(xyFilter, :) / trackedParticlesData.ax_mj(j);
    %             axMnRatio = particles2.ax_mn(xyFilter, :) / trackedParticlesData.ax_mn(j);
    %             ratios = [areaRatio, axMjRatio, axMnRatio]
                % gray scale ratio
                [~, minI] = min(ptToLineDist);

    %             lImg = imread('cam_1_1.tif', 'tiff');
    %             rImg = imread('cam_2_1.tif', 'tiff');
    %             imColorL = cat(3, lImg, lImg, lImg);         
    %             imColorR = cat(3, rImg, rImg, rImg);
    %             [~, loc] = ismember(rightFilteredXY(minI, :), particles2.centroid, 'rows');
    %             p = particles2.bbox(loc, :);
    %             
    %             figure;
    %             hold on;
    % %             subplot(1, 2, 1);
    %             imshow(img1);
    %             for q=1:height(tracks)
    %                 rectangle('Position', tracks.bbox(q, :)', 'FaceColor', 'none', 'EdgeColor', (tracks.colour(q, :)./255)');
    %             end
    %             subplot(1, 2, 2);
    %             imshow(rImg);
    %             rectangle('Position', p', 'FaceColor', 'none', 'EdgeColor', 'blue');
    %             rPtsNew(nLeftParticles, :) = [points(1, :) points(2, :)];
    %             line(rPtsNew(:,[1, 2])',rPtsNew(:,[3, 4])');

    %             xr = particles2
    %             yr = 
                [point3d5, r5] = triangulate(lPts, rightFilteredXY(minI, :), params);
                tracksL.trace{tIdx} = tracksL.trace{tIdx}.updateDetected3D(point3d5);
    %             close all;
    %             tracks.rightCoord2D(rowsToUpdate, :) = particles2(, :); % px
    %             tracks.worldCoordinates(rowsToUpdate, :) = point3d5;
            end
%             trackIdx = 6;
%             showTrack = false;
%             if ((showTrack == true) && (height(tracksL) >= trackIdx))
%                 figure;
%                 hold on;
%         %             subplot(1, 2, 1);
%                 imshow(img1);
%                 tTemp = tracksL; %(assignments(:, 1), :);
%         %         for q=1:height(tTemp)
%                     rectangle('Position', tTemp.bbox(trackIdx, :)', 'FaceColor', 'none', 'EdgeColor', (tracks.colour(5, :)./255)');
%         %         end
% 
%                 close all
%             end
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
    tracksL = mergeLostTracks(tracksL, lostTracksL);
    tracksR = mergeLostTracks(tracksR, lostTracksR);
    tracks = struct('tracksL', tracksL, 'tracksR', tracksR);

    save(computedTrackPath, 'tracks');
    % save('lostTracks.mat', 'lostTracks');
    % writetable(tracks, 'tracks.dat');
    % figure; 
    % hold on;
    % imshow(imColor);
    % 
    % for i=1:height(tracks)
    % %     imColor(floor(tracks.centroid(i, 2)), floor(tracks.centroid(i, 2)), :) = [0, 0, 255];
    %     rectangle('Position', tracks.bbox(i, :)', 'FaceColor', 'none', 'EdgeColor', 'blue');
    % end
    % 
    % hold off;
%     function trackImg = drawTrace(img, tracks)
%         text_str = cell(height(tracks),1);
%         pos = zeros(height(tracks), 2);
% 
%         figure;
%         hold on;
%         % tracks(tracks.totalVisibleCount ~= 2, :) = [];
%         for i=1:height(tracks)
%             text_str{i} = ['T: ' num2str(tracks.id(i), '%d')];
%             trace = tracks.trace{i}.detectedTrace;
%             pos(i, :) = trace(1, :);
%         end
%         trackImg = insertText(img, pos-25, text_str, 'FontSize', 18, ...
%             'BoxColor', tracks.colour, 'BoxOpacity', 0.4, ...
%             'TextColor', 'white');
%         imshow(trackImg, [0 255]);
%         for i=1:height(tracks)
%             trace = tracks.trace{i}.detectedTrace; 
%             line(trace(:, 1), trace(:, 2), ...
%                 'Color', tracks.colour(i, :)./255, ...
%                 'LineWidth', 1);
%         end
%     end


end