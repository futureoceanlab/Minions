function points2d = projectPoints(points3d, P)
    % points3D = points in 3D space
    % P = projection matrix
    points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
    points2dHomog = P * points3dHomog;
    points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end

