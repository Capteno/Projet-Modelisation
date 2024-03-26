function c = contour(img)
   
    % Inversion de l'image
    img = img == 0;
    
    %% Récupération des points sur le contour
    
    % Récupération de la taille de l'image :
    [hauteur, ~] = size(img);
    
    % Initialisation du point pour trouver un point sur le contour
    col = 1;
    row = floor(hauteur/2);
    
    % Initialisation de la variable de contrôle de la boucle
    exit = true;
    
    % Boucle pour trouver un point sur le contour
    while exit && row <= size(img,1) && col <= size(img,2)
        if img(row, col) == true
            exit = false;
        else
            col = col + 1;
        end
    end
    
    % Réalisation du contour
    c = bwtraceboundary(img,[row col],'W');
end

