function figTrace = drawTraceFigure(tracks, img, figTitle)

    startIdx = 1;
    endIdx = height(tracks);
    text_str = cell(endIdx-startIdx+1,1);
    pos = zeros(endIdx-startIdx+1, 2);
    figTrace = figure;
    hold on;
    title(figTitle);
    for i=startIdx:endIdx %1:height(tracks)
        k = i-startIdx+1;
        text_str{k} = ['T: ' num2str(tracks.id(k), '%d')];
        trace = tracks.trace{k}.detectedTrace2D;
%         boundFilter = trace(1, :) > 25 & trace(1, :) < 2592 & trace(2, :) > 25 & trace(2, :) < 1944;
%         trace(~boundFilter) = [];
        pos(k, :) = trace(1, :);
    end
    img = insertText(img, pos-25, text_str, 'FontSize', 18, ...
        'BoxColor', tracks.colour(startIdx:endIdx, :), 'BoxOpacity', 0.4, ...
        'TextColor', 'white');
    imshow(img, [0 255]);
    for i=startIdx:endIdx
        k = i-startIdx+1;

        trace = tracks.trace{k}.detectedTrace2D; 
        line(trace(:, 1), trace(:, 2), ...
            'Color', tracks.colour(k, :)./255, ...
            'LineWidth', 1);
    end
    hold off;
end