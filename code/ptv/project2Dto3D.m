function points3d = project2Dto3D(points2d, P)
    % points3D = points in 3D space
    % P = projection matrix
    points2d(:, 1:2) = points2d(:, 1:2) .* repmat(points2d(:, 3), [1 2]);
    tPt = (points2d - repmat(P(:, 4)', [size(points2d, 1) 1]));
    points3d = (inv(P(:, 1:3)) * tPt')';
%     points3dHomog = [points2d, ones(size(points2d, 1), 1, 'like', points2d)]';
%     points2dHomog = P * points3dHomog;
%     points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
end
