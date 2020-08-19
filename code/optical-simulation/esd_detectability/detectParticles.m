function [particles] = detectParticles(img, boundBuffer)
    blobAnalyzer = vision.BlobAnalysis;
    blobAnalyzer.MinimumBlobArea = 2;
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

    binImg1 = imbinarize(img);
    [area, centroid, bbox, ax_mj, ax_mn, or, ecc, r2, per] = blobAnalyzer(binImg1);
    boundaryFilter = centroid(:, 1) < boundBuffer |centroid(:, 2) < boundBuffer ...
        | centroid(:, 1) > 2590-boundBuffer | centroid(:, 2) > 1944 - boundBuffer;

    id = 1:size(centroid, 1);
    particles = table(id', area, centroid, bbox, ...
                        ax_mj, ax_mn, or, ecc, r2, per);
    particles.Properties.VariableNames{1} = 'id';
    particles.area = double(particles.area);
    particles(boundaryFilter, :) = [];
   

end