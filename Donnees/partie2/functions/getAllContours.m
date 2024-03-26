function contours = getAllContours()

    % Load des images
    load('mask.mat');

    nb_images = 36;

    % Initialisation de la structure qui stocke les contours
    contours = cell(1, nb_images);

    % Parcours des images et calcul des contours
    for i = 1:nb_images

        img = im_mask(:,:,i);
  
        % Calculer les contours
        c = contour(img);
    
        % Stockage
        contours{i} = c;

        %Affichage du contour
        % figure;
        % imshow(img);
        % hold on;
        % plot(contours{i}(:,2),contours{i}(:,1),'g','LineWidth',2);
        % title("Contour");
        % pause(0.1);

    end



    
end

