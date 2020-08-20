function angRadiance = radiantPower()
% RADIANT POWER
% computation of the radiant intensity profile of the combination
% of the LED and the reflector based on the datasheet provided

efficiency = 0.8;
sigma = 5;

% Values needed to be interpolated for finer computation
%%orange
% cenW =  615;
% lumFlux = 107; % lm
% lumWatt = 683;
% 
% wavelength = 590:5:640;
% waveHighRes = 590:640;
% eff = [0.757 0.694 0.631 0.567 0.503 0.442 0.381 0.323 0.265 0.22 0.175];
% 
% interEff = interp1(wavelength, eff, waveHighRes);
% 
% intensityDist = normpdf(waveHighRes, cenW, sigma);

%% Photored
cenW = 660;
waveHighRes = 640:680;

intensityDist = normpdf(waveHighRes, cenW, sigma);
radiantPower = 0.450;% Watt %*sum(intensityDist);
% % figure; plot(waveHighRes, intensityDist./interEff./683);
% 
% % Convert lumin to watts by undoing spectral distribution and sum
% radiantPower = lumFlux*sum(intensityDist./interEff./lumWatt);


%% Dial Reflector Calculation
% coneAngles = deg2rad([0 5 10 15 20 25 30]);
% coneAngFine = deg2rad(0:30);
% relInt = [1 0.6 0.28 0.12 0.075 0.05 0.01];

%% Roithner Reflector Calculation
coneAngles = deg2rad([0 2 4 6 8 10 12 14 16]);
coneAngFine = deg2rad(0:30);
relInt = [1 0.9 0.7 0.3 0.2 0.15 0.1 0.8 0.07];

%% Calculation
coneAngles = deg2rad([0 5 10 15 20 25 30]);
coneAngFine = deg2rad(0:30);
relInt = [1 0.6 0.28 0.12 0.075 0.05 0.01];

relIntFine = interp1(coneAngles, relInt, coneAngFine);
%figure; plot(rad2deg([-coneAngFine(end:-1:1), coneAngFine]), [relIntFine(end:-1:1), relIntFine]); title('Relative intensity using reflector'); xlabel('light angle (deg)'); ylabel('Normalized relative intensity (a.u.)');
% Areas of surface of sphere around the apex of cone of coneAngFine 
areas = (2*pi).*(1-cos(coneAngFine));
% Include the first area aside from the diff
angAreas = diff(areas);
% Sum of intensity across the area of each ring
avgRelInt = (relIntFine(1:end-1) + relIntFine(2:end))./2;
angRelativeInt = angAreas.*avgRelInt;
% Deduce the total relative intensity summed over the entire areas
totalRelativeInt = sum(angAreas.*avgRelInt);
% We want to know W/relative from relative Power
unitRelativePower = radiantPower/totalRelativeInt;
% Absolute power per ring
angInt = unitRelativePower*angRelativeInt;
% power intensity of that ring per unit area
angUnitInt = angInt./angAreas;
% power over the entire conic surface given that intensity per unit area
% divided by the angle to sterdian conversion 
% to compute the radiance!
angRadiance = efficiency*angUnitInt.*areas(2:end)./(2*pi*(1-cos(coneAngFine(2:end)))); % W/sterdian
% figure; plot(rad2deg(coneAngFine(2:end)), angRadiance);
end