function [this] = generate3DPts(this)
    %% Sphere configuration
    % Determine the radius of individual spheres
    sRadius = ones(this.nSpheres, 1);
    if (size(this.sRadiusRange) == [1 1])
    % Monotonic
        sRadius = this.sRadius.*this.sRadiusRange(1);
    else
    % Diverse
        for sIdx = 1:size(sRadiusRange, 1)
            % sphereRatio * pConcentration
            sRadius = rand([this.pConcentration(sIdx), 1]) + this.sRadiusRange(sIdx, 1);
            sRadius = (this.sRadiusRange(sIdx, 2) - this.sRadiusRange(sIdx, 1)) * sRadius;
        end
    end
    sRadius = sRadius./1000; % Convert um to mm
    
    % compute number of 3D points per sphere based on the surface area
    sArea = ceil(4*pi*(max(sRadius).^2)./this.desiredResolution);
    nSpherePts = max(defaultSpherePts, sArea);
    
    this.spheres = zeros(this.nSpheres, (nSpherePts+1)^2, 3);
    
    %% Cylinder configuration
    % Determine the radius and height of individual cylinder
    cRadius = ones(this.nCylinders, 1);
    cHeight = ones(this.nCylinders, 1);
    if (this.cRadiusMax == this.cRadiusMin)
    % Monotonic
        cRadius = cRadius.*this.cRadiusMin;
    else
    % Diverse
        cRadiusRange = this.cRadiusMax - this.cRadiusMin;
        cHeightRange = this.cHeightMax - this.cHeightMin;
        cRadius = cRadiusRange * rand([this.nCylinders, 1]) + cRadiusMin;
        cHeight = cHeightRange * rand([this.nCylinders, 1]) + cHeightMin;
    end
    
    % Determine the orientation of individua / cylinder
    yawD = 0; 
    pitchD = 90*rand([nCylinders, 1]) - 45;
    rollD = 90*rand([nCylinders, 1]) - 45;
    
    this.cylinders = zeros(this.nCylinders, nCylinderPts, 3);
    
    %% Generate 3D points of cylinders and spheres
    for i = 1:this.nParticles
        if i <= this.nCylinders
            cylinderPts = generateCylinder(cRadius(i), cHeight(i), nCylinderPts);
            cylinderPts = rotateCloud(cylinderPts, yawD, pitchD(i), rollD(i));
            cylinderPts = bsxfun(@plus, cylinderPts, this.centroids(i, :));
            this.cylinders(i, :, :) = cylinderPts;
        else
            k = i - this.nCylinders;
            spherePts = generateSphere(nSpherePts, this.sRadius(k));     
            spherePts = bsxfun(@plus, spherePts, this.centroids(i, :));
            this.spheres(k, :, :) = spherePts;
        end
    end
end