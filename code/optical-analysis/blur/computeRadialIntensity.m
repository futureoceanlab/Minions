function avgIntensity = computeRadialIntensity(scanRange, centroid, img)
    w = -scanRange:scanRange;
    intensitySum = zeros(scanRange, 1, 'int32');
    count = zeros(scanRange, 1, 'int32');
%     img1 = img;
    for i = 1:length(w)
        x = centroid(1)+w(i);
        for j = 1:length(w)

            y = centroid(2) + w(j);
            if (y <= 0 || y > 1944)
                continue;
            end
            r = (round(sqrt(w(i)^2+w(j)^2)))+1;
            if r <= scanRange
%                 img1(y, x) = 255;
                intensitySum(r) = intensitySum(r) + int32(img(y, x));
                count(r) = count(r) + 1;
            end
        end
    end
    count(count == 0) = 1;
    avgIntensity = intensitySum./count;
end