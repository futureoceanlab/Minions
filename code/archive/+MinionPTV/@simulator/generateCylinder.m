function cloudPts = generateCylinder(radius, height, nPoints)
% Generate a cylinder with custom radius and height centered around (0, 0,
% 0)
    capArea = pi*radius^2;
    bodyArea = 2*pi*radius*height;
    saRatio = (capArea)/(2*capArea + bodyArea);
    nCapPoints = round(nPoints * saRatio);
    A=rand(2, nPoints - nCapPoints*2);
    bodyAngle=(2*pi).*A(1, 1:end); % random angle 
    
    % random points around the cylindrical surface
    xc= radius.*cos(bodyAngle);
    yc= radius.*sin(bodyAngle);
    zc= height.*A(2, 1:end);
    
    % random radius for top/bottom surface
    B=rand(1, nCapPoints);
    capAngle = (2*pi).*B;
    r = sqrt(rand([1, nCapPoints])).* radius;
    xc2 = r.*cos(capAngle); 
    yc2 = r.*sin(capAngle);
    zcBottom = zeros(1, nCapPoints) ;
    zcTop = ones(1, nCapPoints).* height;
    xc = [xc xc2 xc2]';
    yc = [yc yc2 yc2]';
    zc = [zc zcTop zcBottom]' - height/2;
    cloudPts = [xc yc zc];

end