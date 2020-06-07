% function trackImag = drawTraces(img, tracks)
dirPath = 'simulationImg_1_1';
simPath = sprintf('%s/tracksSimulated.mat', dirPath);
comPath = sprintf('%s/tracksComputed.mat', dirPath);
% load(simPath, 'tracks');
load(comPath, 'tracks');
img = imread(sprintf('%s/cam_1_1.tif', dirPath), 'TIFF');
startIdx = 1;
endIdx = height(tracks);
text_str = cell(endIdx-startIdx+1,1);
pos = zeros(endIdx-startIdx+1, 2);

figure;
hold on;
% tracks(tracks.totalVisibleCount ~= 2, :) = [];
for i=startIdx:endIdx %1:height(tracks)
    k = i-startIdx+1;
    text_str{k} = ['T: ' num2str(tracks.id(i), '%d')];
    trace = tracks.trace{i}.detectedTrace;
    boundFilter = trace(1, :) > 50 & trace(1, :) < 2592 & trace(2, :) > 50 & trace(2, :) < 1944;
    trace(~boundFilter) = [];
    pos(k, :) = trace(1, :);
end
img = insertText(img, pos-25, text_str, 'FontSize', 18, ...
    'BoxColor', tracks.colour(startIdx:endIdx, :), 'BoxOpacity', 0.4, ...
    'TextColor', 'white');
imshow(img, [0 255]);
for i=startIdx:endIdx
    trace = tracks.trace{i}.detectedTrace; 
    line(trace(:, 1), trace(:, 2), ...
        'Color', tracks.colour(i, :)./255, ...
        'LineWidth', 1);
end

% end