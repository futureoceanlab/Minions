function [this] = setup(this, stereoCalibrationPath, simConfig)
    %% 1. stereo calibration parameter
    load(stereoCalibrationPath, 'params');
    this.params = params;
    this.cameraMatrix1 = cameraMatrix(params.CameraParameters1, eye(3), [0 0 0]);
    this.cameraMatrix2 = cameraMatrix(params.CameraParameters2, ...
        params.RotationOfCamera2, params.TranslationOfCamera2);
    this.camPoses 
    this.F = params.FundamentalMatrix;
    this.f1 = params.CameraParameters1.FocalLength;
    this.p1 = params.CameraParameters1.PrincipalPoint;
    this.pxPitch = 0.0022;

    this.intrinsics = [f1(1), 0, p1(1);
                  0, f1(2), p1(2);
                  0, 0, 1]; % mm
    this.fStop = 8; % F-stop
    this.fl = f1(1) * pxPitch;
    this.desiredResolution = 0.020; %mm/px
    this.WD = fl * desiredResolution/pxPitch;

    this.imageSizeY = 1944;
    this.imageSizeX = 2592;
    
    %% 2. simulation configuration 
    this.nFrames = simConfig.frames;
    this.pRadius = simConfig.radius;
    this.pConcentration = simConfig.concentration;
    this.pSinkingRate = simConfig.sinkingRate;
    this.pDirection = simConfig.direction;
    this.pVelocity = 
    this.sphereRatio = simConfig.sphereRatio;
    this.nParticles = sum(pConcentration);
    this.tracks = table();
    this.bgBrightness = 50;
    this.nSpheres
    this.sRadiusRange
    this.sRadiusMax
    this.spheres
    this.nCylinders
    this.cRadiusRange
    this.cRadiusMin
    this.HeightRange
    this.cHeightMax;
    this.cylinders    
    this.defaultSpherePts = 20;

    %% 3. paths to various directories for varying purposes
    this.trackFilePath = sprintf('%s/tracksSimulated.mat', this.simDir);
    this.imgDir = sprintf('%s/img', this.simDir);
       
    %% 4. Decide how many spheres and how many cylinders
    this.nSpheres = round(this.nParticles * sphereRatio);
    this.nCylinders = this.nParticles - this.nSpheres;
    
    %% 5. Generate centroids of particles
    this.generateCentroids(-500, 500, -500, 1500, 250, 750);

    %% 6. Create 3D pts of each spheres and cylinders
    this.generate3DPts();
end