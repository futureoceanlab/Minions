function cloudRotPts = rotateCloud(cloudPts, yawDeg, pitDeg, rolDeg)
% rotation yaw, pitch, roll around the origin in degrees
    cloudRotPts = zeros(size(cloudPts));
    yawRad = deg2rad(yawDeg); 
    pitRad = deg2rad(pitDeg); 
    rolRad = deg2rad(rolDeg);
    
    rotYaw = [cos(yawRad) -sin(yawRad) 0; sin(yawRad) cos(yawRad) 0; 0 0 1];
    rotPit = [cos(pitRad) sin(pitRad) 0; 0 1 0; -sin(pitRad) 0 cos(pitRad)];
    rotRol = [1 0 0; 0 cos(rolRad) -sin(rolRad); 0 sin(rolRad) cos(rolRad)];
    rot = rotYaw * rotPit * rotRol;
    
    for i=1:size(cloudPts, 1)
        cloudRotPts(i, :) = rot*(cloudPts(i, :)');
    end
end