function cloudPts = generateSphere(nPoints, radius)
    [x, y, z] = sphere(nPoints);
    cloudPts = [x(:), y(:), z(:)] * radius; % in mm
end