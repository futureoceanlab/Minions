% This is a function to run one evaluation of PTV against simulated
% ground truth result. We are interested in following information:
% a) number of correctly detected particles
%    i) across varying depth (bin of 5mm)
%   ii) varying sizes (bin of 100um)
% b) blob size (both asis and sharpened)
%    i) across varying depth
% c) 3D coordinate error on average
% d) sinking rate on average
% e) (correct) trace error (in px) per track
% 
% The final data is stored onto a file that is later going to be read
% to do meta-evaluation of the PTV across varyious controlled simulations


%     % 2. Comparison with ground truth (simulation) as a function of
%     % 2.x.i  ) # of particles
%     % 2.x.ii ) different size distribution
%     % 2.x.iii) different shape distribution
% 
%     % 2.a) missed detectionas
%     % 2.b) missed correspondence
%     % 2.c) error of 3D location
%     % 2.d) size distribution
%     % 2.e) shape distribution