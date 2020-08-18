function centroids = generateCentroids(nParticles, ...
                        xmin, xmax, ymin, ymax, zmin, zmax)
    % GENERATE CENTROIDS randomly create a 3D point within the given 
    % boundary
    xPts = (xmax-xmin).*rand(nParticles,1) + xmin;
    yPts = (ymax-ymin).*rand(nParticles,1) + ymin;
    zPts = (zmax-zmin).*rand(nParticles,1) + zmin;
    centroids = [xPts yPts zPts];
end