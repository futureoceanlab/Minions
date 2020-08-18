function has3D = find3DSim(params, sL)
    % FIND3DSIM find whether 3D point is supposed to have been visible
    cameraMatrix2 = squeeze(params.camMatrix(2, :, :));
    has3D = 0;
    mTrace = size(height(sL), 1);
    for i=1:height(sL)
        trace3D = sL.trace{i}.detectedTrace3D;
        if isempty(trace3D)
            continue;
        end
        % project the point to the right camera;
        trace2DR = projectPoints(trace3D, cameraMatrix2')';
        % determine if the distance is within the boundary;
        traceFilt = (trace2DR(:, 1) > 0) & (trace2DR(:, 1) < 2592) ...
        & (trace2DR(:, 2) > 0) & (trace2DR(:, 2) < 1944);

        traceDist = vecnorm(trace3D(traceFilt, :) - squeeze(params.camPoses(2, :, :)), 2, 2);
        mTrace(i) = mean(traceDist);
        has3D = has3D + (sum((traceDist > (params.WD-30)) & (traceDist < (params.WD+30))) > 0);
    end
end