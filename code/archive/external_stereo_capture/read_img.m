fileID = fopen('data/image00003_1.bin');
A = fread(fileID);
B = reshape(A, [2592, 1944]);
imshow(B, [0. 255]);