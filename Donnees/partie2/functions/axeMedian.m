function [pointsSquelette, edgesSquelette, fig] = axeMedian(img,densite)

    % Choix de l'algorithme
    algo = "voronoi";
    
    % Inversion de l'image
    img = img == 0;
    
    %% Récupération des points sur le contour
    
    % Récupération de la taille de l'image :
    [hauteur, largeur] = size(img);
    
    
    % Correction: Inversez i et j, car ginput retourne d'abord X (colonnes) puis Y (lignes)
    %[x, y] = ginput(1); % Utilisez x et y pour éviter la confusion
    col = 1;
    row = floor(hauteur/2);
    
    % Initialisation de la variable de contrôle de la boucle
    exit = true;
    
    while exit && row <= size(img,1) && col <= size(img,2)
        if img(row, col) == true
            exit = false;
        else
            col = col + 1;
        end
    end
    
    % Affichage de l'image originale
    % Création de la figure
    fig = figure('visible', 'off');
    subplot(2,2,1);
    imshow(img);
    title("Image originale");
    
    % Réalisation du contour
    contour = bwtraceboundary(img,[row col],'W');
    
    % Affichage du contour
    subplot(2,2,2)
    imshow(img);
    hold on;
    scatter(col,row,50,'filled');
    hold on;
    plot(contour(:,2),contour(:,1),'g','LineWidth',2);
    title("Contour");
    
    %% Récupération de l'axe médian
    
    Y = contour(:,1);
    X = contour(:,2);
    
    Y_densite = Y(1:densite:end);
    X_densite = X(1:densite:end);
    
    % Récupération des points
    [vX, vY] = voronoi(X_densite,Y_densite);
    hold on;
    
    %% Filtrage
    
    edgesXFiltre = [];
    edgesYFiltre = [];
    XFiltre = [];
    YFiltre= [];
    
    
    for i = 1:length(vX)
        % Récupération du premier point
        x1 = abs(floor(vX(1,i)));
        y1 = abs(floor(vY(1,i)));
    
        % Récupération du second point
        x2 = abs(floor(vX(2,i)));
        y2 = abs(floor(vY(2,i)));
        
        if x2 >= largeur || x1 >= largeur || y1 >= hauteur || y2 >= hauteur
            continue;
        elseif inpolygon(x1,y1,X,Y) && inpolygon(x2,y2,X,Y)
            % Les deux points sont dans l'image
            % Ajout de l'arrete des X dans edgesXFiltre
            edgesXFiltre = [edgesXFiltre, [x1; x2]];
    
            % Ajout de l'arrete de Y dans edgesYFiltre
            edgesYFiltre = [edgesYFiltre, [y1; y2]];
    
    
            % On ajoute le point 
            XFiltre = [XFiltre, x1];
            YFiltre = [YFiltre, y1];
        end
    
    end
    
    subplot(2,2,3);
    imshow(img);
    hold on;
    % Affichage de tous les points du squelette
    %scatter(vX, vY,10); 
    
    % Affichage des points à l'intérieur du squelette
    hold on;
    scatter(XFiltre, YFiltre,5, 'filled');
    title("Points du squelette")
    
    subplot(2,2,4);
    imshow(img);
    hold on;
    % Affichage des arêtes entre deux points
        for i = 1:size(edgesXFiltre, 2)
            x = edgesXFiltre(:, i);
            y = edgesYFiltre(:, i);
            plot(x, y, 'b'); % Changez 'b' pour changer la couleur de la ligne
        end
    title("Squelette");

    pointsSquelette = [XFiltre;YFiltre];
    edgesSquelette = [edgesXFiltre, edgesYFiltre];

end

