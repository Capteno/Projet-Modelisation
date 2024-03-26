clear;
close all;
load('mask.mat');
addpath("functions/");

% Choix de l'image
nbrImage = randi([1, 36]);

% Récupération de l'image
img = im_mask(:,:,nbrImage);

for i=1:10
    % Récupération de l'image
    img = im_mask(:,:,i);

    % Calcul de l'axe médian
    [pointsSquelettes, edgesSquelettes,fig] = axeMedian(img,4);

    % Affichage de la figure
    set(fig, 'visible', 'on');
end





