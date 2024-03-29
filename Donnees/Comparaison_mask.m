clear;
close all;
load("mask.mat")
load("mask_maison.mat")

[~, ~, nb_images] = size(mask_maison);

for i = 1:nb_images
    figure;
    subplot(1,2,1);
    imshow(im_mask(:,:,i));
    subplot(1,2,2);
    imshow(mask_maison(:,:,i));
end