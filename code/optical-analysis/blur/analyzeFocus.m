% ANALYZE FOCUS
% Analyze the impact of the change of depth on the blur size of 
% circular disc on the calibration target.
% We used TIScamera with USB3.0 and Edmund Optics Blue Lens
% Ultimately, we are interested in drawing a comparison between
% the empricial measurement and gaussian blur based on an expected
% circle of confusion.

imgDir = "focus_calibration/dark";
paramPath = "focus_calibration/usb3param.mat";
load(paramPath, 'params');
singleParams = params;
listing = dir(imgDir);
r = 40;

%% Data for dark (-500mm to 500 mm)
% Order
dist = [0, -7.5, -10, -15, -20, -30,  7.5, 10, 15, 20, ...
        30]; % cm from the focal plane
% the circular grid center location manually observed
x = [1357 1352 1354 1357 1358 1355 1412 1409 1406 1409 1407];
y = [1050 1016 1016 1015 1049 1043 1018 1019 1070 1071 924];
WD = 248; % working distance in mm
lineStyleOrder = ["-", "--", "-."];

for d = 7:length(dist)
    figure;
    hold on;
    xlabel('pixel offset (px)');
    ylabel('Intensity');
    idx = [3, d+2];
    legendList = [];
    for i=idx %1:length(listing)
        dirItem = listing(i);
        targetFile = startsWith(dirItem.name, "43810451");
        if ~(targetFile)
            continue;
        end
        curImgPath = sprintf("%s/%s", imgDir, dirItem.name);
        imgOri = rgb2gray(imread(curImgPath)); 
        % undistort based on the camera calibration
        img = undistortImage(imgOri, singleParams);
        centroid = [x(i-2) y(i-2)];
        rIn = computeRadialIntensity(r, centroid, img); 
        if i == 3
            % The radial profile at the focused plane
            plot(0:r-1, (255-rIn), lineStyleOrder(1), 'LineWidth', 2);
        else
            % The radial profile at the offset
            plot(0:r-1, (255-rIn), lineStyleOrder(2), 'LineWidth', 2);
        end
        if (i == 3)
            legendList = [legendList, "Focused"];
        else
            legendList = [legendList, "Observed"];
        end
        if (i == 3)
            % Simulation
            COC = 27 * 27 / (8 * (WD - 27))* (abs(dist(idx(2)-2)) / (WD+dist(idx(2)-2)))/0.0022/2;
            img_g = imgaussfilt(img, COC*1.67);

            rInSim = computeRadialIntensity(r, centroid, img_g);
            plot(0:r-1, (255-rInSim), lineStyleOrder(3), 'LineWidth', 2);
            legendList = [legendList, "Simulated"];
        end
    end
    legend(legendList, 'FontSize', 14);
    ax = gca;
    ax.XAxis.FontSize = 14;
    ax.YAxis.FontSize = 14;
    hold off;
end