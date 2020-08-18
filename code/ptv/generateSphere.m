function cloudPts = generateSphere(nPoints, radius)
    % GENERATE SPHERE using the MATLAB sphere function
    [x, y, z] = sphere(nPoints);
    cloudPts = [x(:), y(:), z(:)] * radius; % in mm
end