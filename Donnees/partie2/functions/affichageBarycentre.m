function affichageBarycentre(C_g, i)

load("dino_Ps.mat");
load('mask.mat');

% Nombre d'images utilisées
nb_images = 36; 

% Récupération du barycentre courant
barycentre = C_g(i,:);

% Définition des dimensions de subplot
rows = 6;
cols = 6;

%figure;

for k = 1:nb_images
    % Récupération du contour de l'image courante
    img = im_mask(:,:,k);

    % Récupération de P
    Pk = P{k};

    % Calcul de la projection on obtient (w*x, w*y, w)
    o = Pk * barycentre';
    o = o ./ repmat(o(3,:), 3, 1);

    color = 'rx';
    if img(floor(o(1,:)), floor(o(2,:))) == 0
        color = 'gx';
    end
    
    % Affichage subplot
    %subplot(rows, cols, k);
    figure;
    imshow(im_mask(:,:,k));
    hold on;
    plot(o(2,:), o(1,:), color);
    title("Barycentre n°" + i + " sur l'Image : " + k);
end

end
