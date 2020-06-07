function [lParticles, rParticles] = detectParticles(lImg, rImg, detectionMode, boundBuffer)
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
    if (detectionMode == "adaptive")
        binImg1 = imbinarize(lImg, adaptthresh(lImg, 0.5, 'Statistic', 'gaussian'));
        binImg2 = imbinarize(rImg, adaptthresh(rImg, 0.5, 'Statistic', 'gaussian'));

    else
%         T1 = graythresh(lImg);
%         T2 = graythresh(rImg);
%         sharpImg1 = imsharpen(lImg, 'Radius', 7, 'Amount', 1);
        binImg1 = imbinarize(lImg, 0.2); %, adaptthresh(sharpImg1));
%         sharpImg2 = imsharpen(rImg, 'Radius', 7, 'Amount', 1);
        binImg2 = imbinarize(rImg, 0.2);
    end
%     figure; imshow(sharpImg1);
%     figure; imshow(abs(imbinarize(lImg) - binImg1));
%     figure; imshow(abs(imbinarize(lImg) - binSharpImg1));
%     figure; imshow(imbinarize(lImg));
%     figure; imshow(binImg1);
%     figure; imshow();
    [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(binImg1);
    boundaryFilter = centroid(:, 1) < boundBuffer |centroid(:, 2) < boundBuffer ...
        | centroid(:, 1) > 2590-boundBuffer | centroid(:, 2) > 1944 - boundBuffer;

    id = 1:size(centroid, 1);
    lParticles = table(id', area, centroid, bbox, ...
                        ax_mj, ax_mn, or, ecc, r2, per);
    lParticles.Properties.VariableNames{1} = 'id';
    lParticles.area = double(lParticles.area);
    lParticles(boundaryFilter, :) = [];
    
    % Perform blob analysis to find particles in the right framwe
    [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(binImg2);
    boundaryFilter = centroid(:, 1) < boundBuffer |centroid(:, 2) < boundBuffer ...
        | centroid(:, 1) > 2590-boundBuffer | centroid(:, 2) > 1944 - boundBuffer;
    id = 1:size(centroid, 1);
    rParticles = table(id', area, centroid, bbox, ...
                        ax_mj, ax_mn, or, ecc, r2, per);
    rParticles.Properties.VariableNames{1} = 'id';
    rParticles.area = double(rParticles.area);
    rParticles(boundaryFilter, :) = [];

end