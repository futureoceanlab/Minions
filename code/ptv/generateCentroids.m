function centroids = generateCentroids(nParticles, ...
                                    xmin, xmax, ymin, ymax, zmin, zmax)
    xPts = (xmax-xmin).*rand(nParticles,1) + xmin;
    yPts = (ymax-ymin).*rand(nParticles,1) + ymin;
    zPts = (zmax-zmin).*rand(nParticles,1) + zmin;
    centroids = [xPts yPts zPts];
end