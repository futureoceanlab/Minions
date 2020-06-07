ac = double(imread('cam_1_1.tif', 'TIFF'))./255;
bc = double(imread('bokeh_cam_1_1.tif', 'TIFF'))./255;
figure; imshow(abs(imbinarize(ac) - imbinarize(imsharpen(bc, 'Radius', 10, 'Amount', 2), 'adaptive', 'Sensitivity',0.001)));