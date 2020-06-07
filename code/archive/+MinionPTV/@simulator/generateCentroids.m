function [this] = generateCentroids(this, xmin, xmax, ymin, ymax, zmin, zmax)
    xPts = (xmax-xmin).*rand(this.nParticles,1) + xmin;
    yPts = (ymax-ymin).*rand(this.nParticles,1) + ymin;
    zPts = (zmax-zmin).*rand(this.nParticles,1) + zmin;
    this.centroids = [xPts yPts zPts];
end