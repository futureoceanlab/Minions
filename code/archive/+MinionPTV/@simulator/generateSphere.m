function cloudPts = generateSphere(nPoints, radius)
    % 'sphere' method creates a sphere with radius=1
    [x, y, z] = sphere(nPoints);
    cloudPts = [x(:), y(:), z(:)] * radius; % in mm
end