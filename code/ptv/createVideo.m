function createVideo(curSimImgDir, curTrackEvalDir, matchIdx, tL, tIdx, sL, sIdx)
    dt = tL.trace{tIdx}.detectedTrace2D(matchIdx(sIdx, 1):matchIdx(sIdx, 2), :);
    st = sL.trace{sIdx}.detectedTrace2D(matchIdx(sIdx, 3):matchIdx(sIdx, 4), :);
    v = VideoWriter(sprintf("%s/trace_%d_%d.avi", curTrackEvalDir, sIdx, tIdx));
    v.FrameRate = 4;
    open(v);
    fIdx = matchIdx(sIdx, 5);

    lenMatch = matchIdx(sIdx, 2) - matchIdx(sIdx, 1);
    for l=1:lenMatch
        img1Path = sprintf('%s/cam_1_%d.tif', curSimImgDir,fIdx);
        img1 = imread(img1Path, 'TIFF');
        d = round(tL.ax_mj(tIdx));
        txy = dt(l, :) - d/2;
        img1 = insertShape(img1, 'Rectangle', [txy d d], ...
            'Color','red', 'LineWidth', 2);
        sxy = st(l, :) - d/2;
        img1 = insertShape(img1, 'Rectangle', [sxy d d], ...
            'Color','yellow', 'LineWidth', 2);
        writeVideo(v, img1);
        fIdx = fIdx + 1;
    end
    close(v);
end