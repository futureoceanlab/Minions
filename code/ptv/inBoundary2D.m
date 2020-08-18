function val = inBoundary2D(pts, minX, maxX, minY, maxY)
    % IN BOUNDARY 2D check that a 2D point is within the boundary provided
    val = (pts(1) >= minX) & (pts(2) >= minY) ...
               & pts(1) <= maxX & (pts(2) <= maxY);
end