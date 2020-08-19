imgDir = "focus_calibration/dark";
paramPath = "focus_calibration/usb3param.mat";
load(paramPath, 'params');
singleParams = params;
listing = dir(imgDir);
r = 40;
%% Initial data in bright / motion between -5 ~ 5 mm
% Order
% -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5
% points = [1384 1403; 1387 1405; 1388 1405; 1337 1352; 1338 1352; 1339 1352; 1392 1406; 1340 1355; 1339 1356; 1340 1358; 1339 1361];
% x = round(sum(points, 2)./2);
% y = [857; 1011; 1062; 1011; 958; 1010; 956; 1010; 1010; 1063; 1009];

%% Data for dark (-500mm to 500 mm)
% Order
% 0mm, -75, -100, -150, -200, -300, -400, -500, 75, 100, 150, 200, 300,
% 400, 500
dist = [0, -7.5, -10, -15, -20, -30,  7.5, 10, 15, 20, ...
        30];
x = [1357 1352 1354 1357 1358 1355 1412 1409 1406 1409 1407];
y = [1050 1016 1016 1015 1049 1043 1018 1019 1070 1071 924];
WD = 248;
lineStyleOrder = ["-", "--", "-."];

for d = 7:length(dist)
figure;
hold on;
% title(sprintf('Depth Offset from Focal Plane = %.1f', dist(d)));
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
    
%     dirItem.name
    curImgPath = sprintf("%s/%s", imgDir, dirItem.name);
    imgOri = rgb2gray(imread(curImgPath)); 
    img = undistortImage(imgOri, singleParams);
%     figure; imshow(img, [0 255]);

    centroid = [x(i-2) y(i-2)];
    rIn = computeRadialIntensity(r, centroid, img); 
    if i == 3
        plot(0:r-1, (255-rIn), lineStyleOrder(1), 'LineWidth', 2);
    else
        plot(0:r-1, (255-rIn), lineStyleOrder(2), 'LineWidth', 2);
    end
    if (i == 3)
        legendList = [legendList, "Focused"];
    else
        legendList = [legendList, "Observed"];
    end
%     plot(-r:r, (255 - img(y(i-2), x(i-2)-r:x(i-2)+r)));
       
%     imshow(img);
    if (i == 3)
        COC = 27 * 27 / (8 * (WD - 27))* (abs(dist(idx(2)-2)) / (WD+dist(idx(2)-2)))/0.0022/2
        filtSize = 2*ceil(2*COC/2)+1; %1.67
        if (mod(filtSize, 2) == 0)
            filtSize = filtSize - 1;
        end
        img_g = imgaussfilt(img, COC*1.67); %, 'FilterSize', filtSize); 1.67

        rInSim = computeRadialIntensity(r, centroid, img_g);
        plot(0:r-1, (255-rInSim), lineStyleOrder(3), 'LineWidth', 2);
        legendList = [legendList, "Simulated"];
%         plot(-r:r, (255 - img_g(y(i-2), x(i-2)-r:x(i-2)+r)));
    end
%     close all;
end
% if (d == 7)
    legend(legendList, 'FontSize', 14);
% end
ax = gca;
ax.XAxis.FontSize = 14;
ax.YAxis.FontSize = 14;
hold off;
end