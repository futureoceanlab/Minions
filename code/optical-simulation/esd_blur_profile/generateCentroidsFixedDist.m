function centroids = generateCentroidsFixedDist(nParticles, ...
                        xmin, xmax, ymin, ymax, depth)
    xPts = (xmax-xmin).*rand(nParticles,1) + xmin;
    yPts = (ymax-ymin).*rand(nParticles,1) + ymin;
    zPts = sqrt(depth^2 - (xPts.^2 + yPts.^2));
    centroids = [xPts yPts zPts];                    
end