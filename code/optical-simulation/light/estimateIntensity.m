%% 0. Left camera position
load('params.mat', 'params');
cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
    params.RotationOfCamera2, params.TranslationOfCamera2);

camMatrix = zeros([2, size(cameraMatrix1)]);
camMatrix(1, :, :) = cameraMatrix1;
camMatrix(2, :, :) = cameraMatrix2;
    
[~, lCameraPos] = extrinsicsToCameraPose(eye(3),...
                                        zeros(1, 3));
[rOrientation, rCameraPos] = extrinsicsToCameraPose(params.RotationOfCamera2,...
                                        params.TranslationOfCamera2);
camPoses = [lCameraPos; rCameraPos];
desiredResolution = 0.020;
pxPitch = 0.0022;
focalLen = params.CameraParameters1.FocalLength(1) * pxPitch;
WD = focalLen * desiredResolution/pxPitch + focalLen;
N = 8;
cL = [0, 0, 0];


%% 2. Light in P1 and P2 at varying angle
b = 80; % mm half of the baseline
tPoint = [0, 0, WD];
delta = deg2rad(25);
dist = 200:50:400; % mm light point distance away from target center
phi = deg2rad(10:10:90); % rad - elevation angle of the light
lightLocations = zeros(size(dist, 2), 2, 3, size(phi, 2));
locDir = [-1 1];
for dIdx=1:size(dist, 2)
    d = dist(dIdx);
    for i=1:2
        lTempPoint = [zeros(1, size(phi, 2)); locDir(i)*d*sin(phi); -d*cos(phi)];

        % lRot = [1 0 0; 0 cos(delta) -sin(delta); 0 sin(delta) cos(delta)];
        lRot = [cos(delta) 0 sin(delta); 0 1 0; -sin(delta) 0 cos(delta)];
        repLRot = repmat(lRot, 1, size(lTempPoint, 2));
        lPoints = zeros(size(lTempPoint));
        for aIdx = 1:size(lTempPoint, 2)
            lPoints(:, aIdx) = (lTempPoint(:, aIdx)'*lRot)';
        end
        lPoints = lPoints + repmat(tPoint', 1, size(lTempPoint, 2));
        lightLocations(dIdx, i, :, :) = lPoints;
    end
end
% lPoints([1 2 3], :) = lPoints([1 3 2], :); % Just for visualization swap z
% % and y
% figure; hold on;
% plot3(lPoints(1, :),lPoints(3, :),lPoints(2, :), 'o');
% plot3(tPoint(1),tPoint(3),tPoint(2), 'o', 'Color', 'r');


%% 4. Visible contours of spheres 
%   They are defined by comparing points on spheres
%   to a plane across those contours. We take the inner product between
%   the plane and the normal towards the camera.
fPoint = [0; 0; focalLen];
exposure = 1e-3;
QE = 0.8;
gain = 8;
responsivity = 1.4; % V/lux-sec
Fstop = 8;
aperture = pi*((focalLen/Fstop)/2)^2;
lumWattConversion = 683;
Vss =  3.1;
attnCoeff = 0.25;
albedo = 0.8; 
imageSizeY = 1944;
imageSizeX = 2592;
img = zeros(imageSizeY, imageSizeX);

%% 1. Spheres in 3D %0.05,
sRadius = [0.05, 0.1, 0.25,0.5]; % mm 
sCenters = [-30 -15 WD+20;
            -15 -15 WD-20;
            -25 15 WD+20;
            -8 15 WD-20;
            0 0 WD;
            5 15 WD+20;
            10 15 WD-20;
            10 -15 WD+20;
            20 -15 WD-20;]';
nSpheres = size(sCenters, 2);

lSum = zeros(size(sRadius, 2), size(dist, 2), size(phi, 2));
cDiff = zeros(size(sRadius, 2), size(dist, 2),size(phi, 2), 2);
cDiffStd = zeros(size(sRadius, 2), size(dist, 2),size(phi, 2), 2);
pxSum = zeros(size(sRadius, 2), size(dist, 2), size(phi, 2));

for rIdx=1:size(sRadius, 2)
    nPoints = max(sRadius(rIdx)/0.2, 1) * 100;
    r = sRadius(rIdx);
    spheres = zeros(nSpheres, (nPoints+1)*(nPoints+1), 3);
    for sIdx=1:nSpheres
        [xs, ys, zs] = sphere(nPoints);
        spherePts = [xs(:), ys(:), zs(:)] * r;
    %     spherePts = sphere(cloudPts)*r;
        spheres(sIdx, :, :) = bsxfun(@plus, spherePts, sCenters(:, sIdx)');
    end
    for dIdx=1:size(dist, 2)
        for aIdx = 1:size(phi, 2)
            imgTemp = zeros(2, imageSizeY, imageSizeX);
            img2 = zeros(imageSizeY, imageSizeX);

            for sIdx=1:nSpheres
                
                c = sCenters(:, sIdx);
                fcLine = fPoint - c; % define the normal to the plane of interest
                fcDist = norm(fcLine);
                normFn = fcLine./fcDist;
                interPoint = c + normFn*(r^2/fcDist);

                sPoints = squeeze(spheres(sIdx, :, :));
                vPoints = zeros(size(sPoints));
                for pIdx =1:size(sPoints)
                    visible = dot(normFn, sPoints(pIdx, :)'-interPoint) > 0;
                    vPoints(pIdx, :) = visible*sPoints(pIdx, :);
                end
                vPoints(sum(vPoints==[0 0 0], 2)==3, :) = [];
            %     plot3(vPoints(:, 1), vPoints(:, 3), vPoints(:, 2), '.', 'Color', 'g');

        %         vvPoints = zeros(size(vPoints));
                pImgTemp = zeros(2, imageSizeY, imageSizeX);

                for lIdx = 1:2
                    lPts = squeeze(lightLocations(dIdx, lIdx, :, aIdx))';
                    angRadiance = radiantPower();
                    lightDirectionalVec = lPts - [0 0 WD]; %tPoint;
                    lightDirectionalVec = lightDirectionalVec / norm(lightDirectionalVec);
                    pts = round(projectPoints(vPoints, squeeze(camMatrix(1, :, :))'))+1;
                    COC = focalLen * focalLen / (Fstop * (WD - focalLen))* (abs(fcDist+focalLen - WD) / (fcDist+focalLen))/pxPitch/2;
                    ptsMax = max(1, min([imageSizeX; imageSizeY], max(pts, [], 2) + floor(3*COC)));
                    ptsMin = min([imageSizeX; imageSizeY], max(1, min(pts, [], 2) - floor(3*COC)));

                    for vIdx=1:size(vPoints, 1)
            %             for lightIdx=-1:2:1
                            lPtsTemp = [lPts(1) lPts(2) lPts(3)];
                            nVec = vPoints(vIdx, :) - c';
                            nVec = nVec/norm(nVec);
                            lsVecReflected = -lightDirectionalVec+ nVec*(dot(nVec, lightDirectionalVec)*2);
                            N = cross(lightDirectionalVec, nVec);
                            N = N/norm(N);

                            fsVec = fPoint' - vPoints(vIdx, :);
                            lnAngle = atan2(norm(cross(lightDirectionalVec,nVec)), dot(lightDirectionalVec,nVec));
                            if (cos(lnAngle) > 0)
                    %             projFS = fsVec - do(fsVec, N)*N;
                    %             alpha = abs( pi/2 - acos( dot(fsVec, N)/norm(N)/norm(fsVec) ) );

            %                     vvPoints(vIdx, :) = vPoints(vIdx, :);
            %                     nfAngle = atan2(norm(cross(fsVec,nVec)), dot(fsVec,nVec));
                                nfAngle = atan2(norm(cross(fsVec, lsVecReflected)), dot(fsVec, lsVecReflected));
        %                         csAngle =cos(nfAngle);% - lnAngle)); % cos(alpha)*
                                x = pts(1, vIdx);
                                y = pts(2, vIdx);
                                if inBoundary2D(pts(:, vIdx), 1, imageSizeX, 1, imageSizeY)
                                    lsVec = lPtsTemp - vPoints(vIdx, :);
                                    fsAngle = atan2(norm(cross(fsVec, fPoint)), dot(fsVec, fPoint));
                                    reflectorAngle = rad2deg(atan2(norm(cross(lightDirectionalVec,lsVec)), dot(lightDirectionalVec,lsVec)));
                                    radiance = exp(-attnCoeff*norm(lsVec)*1e-3)*cos(lnAngle)*angRadiance(ceil(reflectorAngle))/(norm(lsVec)^2)*(0.02^2); % multiply by sterdian
                                    targetToLensLoss = exp(-attnCoeff*norm(fsVec*1e-3))*aperture/(norm(fsVec)^2);
                                    targetReflection = albedo*cos(nfAngle)*(cos(fsAngle)^4)*targetToLensLoss*radiance;
                                    sensorVoltage = targetReflection*exposure*QE*gain/((2.2e-6)^2)*responsivity*lumWattConversion;
                                    if (sensorVoltage > 0)
                                        pImgTemp(lIdx, y, x) = sensorVoltage;
                                    end
        %                             end
                                end
                            end
            %             end
                    end
                end

                pImgTemp(pImgTemp < 0 ) = 0;
                pImg = squeeze(pImgTemp(1, :, :)) + squeeze(pImgTemp(2, :, :));
%                 pImg(pImg > 1 ) = 1;
%                 if COC ~= 0
%                     pImg = imgaussfilt(pImg(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)), COC);
%                 else
                    pImg = pImg(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1));
%                 end
%                 imwrite(pImg, sprintf('single_esd_%d_angle_%d.tif', r*2000, round(rad2deg(phi(aIdx)))));
        %                 imgTemp2 = imgTemp(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1));
                img(ptsMin(2):ptsMax(2), ptsMin(1):ptsMax(1)) = pImg;
        %         imgPair(camI, :, :) = squeeze(imgPair(camI, :, :)) + imgTemp;
        %                             if (iVal > 0) && (iVal < 1)
        %         imgTemp(lIdx, y, x) = sensorDigit;%imgTemp(y, x) +
        %         img(img < 1) = 1;
                pts2 = round(projectPoints(sPoints, squeeze(camMatrix(1, :, :))'))+1;
                for j=1:size(sPoints, 1)
                    x = pts2(1, j);
                    y = pts2(2, j);
                    if inBoundary2D(pts2(:, j), 1, imageSizeX, 1, imageSizeY)
                        img2(y, x) = 1;%imgTemp(y, x) +
                    end
                end
        %     vvPoints(sum(vvPoints==[0 0 0], 2)==3, :) = [];
        %     plot3(vvPoints(:, 1), vvPoints(:, 3), vvPoints(:, 2), '.', 'Color', 'g');
        %     figure; imshow(imgTemp2);
            end
%             scenario = sprintf('esd_%d_angle_%d_dist_%d', r*2000, round(rad2deg(phi(aIdx))), dist(dIdx));
%             imwrite(img, sprintf('%s.tif', scenario)); 
%         %     imgTemp(imgTemp < 0 ) = 0;
%         %     img = squeeze(imgTemp(1, :, :)) + squeeze(imgTemp(2, :, :));
%         %     imgTemp(imgTemp > 1 ) = 1;
            lSum(rIdx, dIdx, aIdx) = sum(img, 'all');
            pxSum(rIdx, dIdx, aIdx) = sum(img2, 'all');

%             img = img / Vss;
%             img(img > 1) = 1;
%             img = img * 255;
%             img(img > 255) = 255;
%             img(img < 1) = 1;
%             img = rescale(log2(img));
% 
% 
%             particlesLights = detectParticles(img, 0);
%             particlesAmbient = detectParticles(img2, 0);
% 
%             cDiff(rIdx, dIdx, aIdx, :) = mean(abs(particlesLights.centroid - particlesAmbient.centroid));
%             cDiffStd(rIdx, dIdx, aIdx, :) = std(abs(particlesLights.centroid - particlesAmbient.centroid));
        end
    end
end
save('lSum3.mat', 'lSum');
save('cDiff3.mat', 'cDiff');
save('cDiffStd3.mat', 'cDiff');
save('pxSum3.mat', 'pxSum');
figure; hold on; 
title('');
plot(rad2deg(phi), lSum./max(lSum));
xlabel('light angle (deg)');
ylabel('normalized total intensity (a.u.)');

figure; hold on; plot(rad2deg(phi), cDiff(:, 1)); plot(rad2deg(phi), cDiff(:, 2)); plot(rad2deg(phi), mean(cDiff, 2));
% 5. Light intensity is projected onto the visible surfaces of the spheres


% 6. Each luminous points on the spheres are projected to image sensor
%    with provided luminosity

% figure; hold on; plot(rad2deg(phi), squeeze(mean(squeeze(cDiffStd(:, :, :, 2)), 2))); xlabel("Light Angle (deg)"); ylabel("y-axis Detection Offset Error (px)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]); ylim([0 0.6]);
% figure; hold on; plot(rad2deg(phi), squeeze(mean(squeeze(cDiffStd(:, :, :, 2)), 2))); xlabel("Light Angle (deg)"); ylabel("y-axis Detection Offset Error (px)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]); ylim([0 0.6]);
% figure; hold on; plot(rad2deg(phi), squeeze(mean(squeeze(cDiffStd(:, :, :, 1)), 2))); xlabel("Light Angle (deg)"); ylabel("x-axis Detection Offset Error (px)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]); ylim([0 0.6]);
% figure; hold on; plot(rad2deg(phi), squeeze(mean(squeeze(cDiff(:, :, :, 1)), 2))); xlabel("Light Angle (deg)"); ylabel("x-axis Detection Offset (px)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]); ylim([0 3.5]);
% figure; hold on; plot(rad2deg(phi), squeeze(mean(squeeze(cDiff(:, :, :, 2)), 2))); xlabel("Light Angle (deg)"); ylabel("y-axis Detection Offset (px)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]); ylim([0 3.5]);
% figure; plot(rad2deg(phi), squeeze(lSum(:, 4, :)./pxSum(:, 4, :)));xlabel("Light Angle (deg)"); ylabel("Voltage at the pixel (V)"); legend(["ESD=100um", "ESD=200um", "ESD=500um", "ESD=1000um"]);
