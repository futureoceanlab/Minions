function [lParticles, rParticles] = detectParticles(lImg, rImg)
    blobAnalyzer = vision.BlobAnalysis;
    blobAnalyzer.MinimumBlobArea = 1;
    blobAnalyzer.MaximumBlobArea = 20000;
    blobAnalyzer.MaximumCount = 100;
    blobAnalyzer.AreaOutputPort = true;
    blobAnalyzer.CentroidOutputPort = true;
    blobAnalyzer.BoundingBoxOutputPort = true;
    blobAnalyzer.MajorAxisLengthOutputPort = true;
    blobAnalyzer.MinorAxisLengthOutputPort = true;
    blobAnalyzer.OrientationOutputPort = true;
    blobAnalyzer.EccentricityOutputPort = true;
    blobAnalyzer.EquivalentDiameterSquaredOutputPort = true;
    blobAnalyzer.PerimeterOutputPort = true;
%     sharpImg1 = imsharpen(lImg, 'Radius', 7, 'Amount', 1);
%     binSharpImg1 = imbinarize(sharpImg1); %, adaptthresh(sharpImg1));
    binImg1 = imbinarize(lImg, adaptthresh(lImg, 0.4, 'Statistic', 'gaussian'));
%     figure; imshow(sharpImg1);
%     figure; imshow(abs(imbinarize(lImg) - binImg1));
%     figure; imshow(abs(imbinarize(lImg) - binSharpImg1));
%     figure; imshow(imbinarize(lImg));
%     figure; imshow(binImg1);
%     figure; imshow();
    [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(binImg1);
    id = 1:size(centroid, 1);
    lParticles = table(id', area, centroid, bbox, ...
                        ax_mj, ax_mn, or, ecc, r2, per);
    lParticles.Properties.VariableNames{1} = 'id';
    lParticles.area = double(lParticles.area);

    % Perform blob analysis to find particles in the right framwe
    [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(imbinarize(rImg));
    id = 1:size(centroid, 1);
    rParticles = table(id', area, centroid, bbox, ...
                        ax_mj, ax_mn, or, ecc, r2, per);
    rParticles.Properties.VariableNames{1} = 'id';
    rParticles.area = double(rParticles.area);
end