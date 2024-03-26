function [tetraedres] = triTetraedres(C_g)

load("dino_Ps.mat");
load('mask.mat');

% Nombre d'images utilisees
nb_images = 36; 

% Récupération de tout les contours
% contours = getAllContours();

% Création des variables qui contient les points et les arretes
tetraedres = [];
indicesImg = [];

% Récupération du nombre de barycentre 
nb_barycentres = size(C_g,1);

h = waitbar(0, 'Traitement en cours...');
tic

% On regarde si chaque barycentre est bien dans une figure
for i = 1:nb_barycentres

    waitbar(i/nb_barycentres, h, sprintf('Traitement en cours... %d%%', floor((i/nb_barycentres) * 100)));

    % Récupération du barycentre courant
    barycentre = C_g(i,:);

    for k=1:nb_images
        % Récupération du contour de l'image courante
        img = im_mask(:,:,k);

        % Récupération de P
        Pk = P{k};

        % Calcul de la projection on obtient (w*x, w*y, w)
        o = Pk * barycentre';

        % On divise par w
        o = o ./ repmat(o(3,:), 3, 1);
       
        % Condition
        if img(floor(o(1,:)), floor(o(2,:))) == 1
            % Alors on ajoute le tétraèdre
            tetraedres = [tetraedres i];            
            break

        end


    end


end

close(h);
toc

end

