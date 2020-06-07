function val = inBoundary(pts, minX, maxX, minY, maxY)
    val = (pts(1) >= minX) & (pts(2) >= minY) ...
               & pts(1) <= maxX & (pts(2) <= maxY);
end
