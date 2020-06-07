function [this] = runSimulation(this)
% Author: Junsu Jang (junsu.jang94@gmail.com)
    fprintf("creating the simulation image...\n");
    for frameI=1:this.nFrames
        for i=1:this.nParticles
            imgTemp = zeros(imageSizeY, imageSizeX);
            wasInFrame = [0 0];

            if frameI==1
                trace = MinionPTV.bricks.particleTrace;
                newTrack = table(i, 0, 0, 0, 0, ...
                     255*rand(3, 1)', [NaN NaN], [NaN NaN NaN],...
                     [NaN NaN], {trace}, ...
                    'VariableNames', {'id', 'age',  'totalVisibleCount', ...
                    'totalInvisibleCount', 'consecutiveInvisibleCount', 'colour', ...
                    'centroid', 'worldCoordinates', 'rightCoord2D', 'trace'
                });
                this.tracks = [this.tracks; newTrack];
            end
            for camI=1:2
                if (i <= this.nCylinders)
                    curPts = squeeze(this.cylinders(i, :, :));
                else 
                    k = i - this.nCylinders;
                    curPts = squeeze(this.spheres(k, :, :));
                end
                pts = round(bricks.projectPoints(curPts, squeeze(this.camMatrix(camI, :, :))'))+1;
                coord2D(camI, i, :) = mean(pts, 2);
                coord3D(i, :) = mean(curPts)';
                dist = norm(this.centroids(i, :) - this.camPoses(camI, :));
                COC = (this.fl^2) / (this.fStop * (this.WD - this.fl))* (abs(dist - this.WD) / dist)/this.pxPitch/2;
                ptsMax = max(1, min([this.imageSizeX; this.imageSizeY], max(pts, [], 2) + floor(3*COC)));
                ptsMin = min([this.imageSizeX; this.imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

                for j = 1:size(pts, 2)
                    x = pts(1, j);
                    y = pts(2, j);
                    if bricks.inBoundary(pts(:, j)', 1, imageSizeX, 1, imageSizeY)
                        imgTemp(y, x) = 255;
                        wasInFrame(camI) = 1;
                    end
                end
                imgTemp2 = imgaussfilt(imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), COC);
    %             imgTemp2 = imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1));
    % % 
                imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = imgTemp2;
                imgPair(camI, :, :) = squeeze(imgPair(camI, :, :)) + imgTemp;
            end
            % Update the track
            if wasInFrame(1) == 1
                this.tracks.centroid(i, :) = squeeze(coord2D(1, i, :));
                if wasInFrame(2) == 1
                    % In both left and right frame
                    this.tracks.rightCoord2D(i, :) = squeeze(coord2D(2, i, :));
                    this.tracks.worldCoordinates(i, :) = coord3D(i, :);
                end
                this.tracks.age(i) = this.tracks.age(i) + 1;
                this.tracks.totalVisibleCount(i) = this.tracks.totalVisibleCount(i) + 1;
                this.tracks.trace{i} = this.tracks.trace{i}.addDetected(squeeze(coord2D(1, i, :)));
            else
                % Invisible
                if this.tracks.age(i) > 0
                    this.tracks.age(i) = this.tracks.age(i) + 1;
                    this.tracks.totalInvisibleCount(i) = this.tracks.totalInvisibleCount(i) + 1;
                    this.tracks.consecutiveInvisibleCount(i) = this.tracks.consecutiveInvisibleCount(i) + 1;
                end
            end
        end
        img1Path = sprintf('%s/cam_1_%d.tif', this.imgDir, frameI);
        img2Path = sprintf('%s/cam_2_%d.tif', this.imgDir, frameI);
        imwrite(squeeze(imgPair(1, :, :))./255, img1Path, 'TIFF');
        imwrite(squeeze(imgPair(2, :, :))./255, img2Path, 'TIFF');

        imgPair = zeros(2, this.imageSizeY, this.imageSizeX) + this.bgBrightness;
        this.cylinders(:, :, 2) = this.cylinders(:, :, 2) + this.cVelocity;
        this.spheres(:, :, 2) = this.spheres(:, :, 2) + this.sVelocity;
    end
end