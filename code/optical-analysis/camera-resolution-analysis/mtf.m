function [cont, res] = mtf(target, cx, cy, windowSize)
    % MTF compute the MTF across the radial intensity of the target 
    
    % Divide into n_seg segments
    n_seg = 24*4;
    seg_rad = 2*pi/n_seg;
    cont = zeros(91, 1);
    res = zeros(141, 1);
    % We need to skip the empty 200um center, so start from r=10
    for r=10:1:204
        segs = zeros(n_seg, 1);
        for seg=0:(n_seg-1)
            seg_x = cy+round(r*sin(seg_rad*seg));
            seg_y = cx+round(r*cos(seg_rad*seg));
            segs(seg+1) = target(seg_x, seg_y);
        end
        % Convert to resolution
        res(r-9) = 2*pi*r*0.02/72;
        segs_mean = mean(segs);
        % Compute the contrast
        a = abs(segs-segs_mean);
        cont(r-9) = max(a)/segs_mean;
    end
    % Filter out using a window size of 5
    b = (1/windowSize)*ones(1,windowSize);
    a = 1;
    cont = filter(b,a,cont);
end
