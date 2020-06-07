function lr = computeLinearRegression(tracks)
    lr = zeros(height(tracks), 4);
    for k=1:length(lr)
        dT = tracks.trace{k}.detectedTrace;
        Y = [ones(length(dT),1) dT(:, 2)];
        X = [ones(length(dT),1) dT(:, 1)];
        rank(Y)
        bY = Y\dT(:, 1);
        bX = X\dT(:, 2);
        if rank(bY) == 1
            m
        lr(k, :) = [bY' bX'];
    end
end