clear;
close all;
% Nombre d'images utilisees
nb_images = 36; 

% chargement des images
for i = 1:nb_images
    if i<=10
        nom = sprintf('images/viff.00%d.ppm',i-1);
    else
        nom = sprintf('images/viff.0%d.ppm',i-1);
    end
    % im est une matrice de dimension 4 qui contient 
    % l'ensemble des images couleur de taille : nb_lignes x nb_colonnes x nb_canaux 
    % im est donc de dimension nb_lignes x nb_colonnes x nb_canaux x nb_images
    im(:,:,:,i) = imread(nom); 
end

% Affichage des images
figure; 
subplot(2,2,1); imshow(im(:,:,:,1)); title('Image 1');
subplot(2,2,2); imshow(im(:,:,:,9)); title('Image 9');
subplot(2,2,3); imshow(im(:,:,:,17)); title('Image 17');
subplot(2,2,4); imshow(im(:,:,:,25)); title('Image 25');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A COMPLETER                                             %
% Calculs des superpixels                                 % 
% Conseil : afficher les germes + les régions             %
% à chaque étape / à chaque itération                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%========== Chargement de l'image ==========%

imagetest = im(:,:,:,1);
[hauteur, largeur, ~] = size(imagetest);
mat = rgb2lab(imagetest);
[matPositionLargeur, matPositionHauteur] = meshgrid(1:largeur, 1:hauteur);
mat = cat(3, mat, matPositionHauteur, matPositionLargeur);



%========== Variables ==========%

k = 400; % Nombre de superpixel
m = 30; % Compacité
n = 3; % Taille du voisinnage pour l'affinement de la grille
nbIterSPmax = 5; % Itération max de k-means (Réduction du temps de calcul)
Seuil = 10; % Seuil de fin de k-means
voisCC = 2; % Taille du voisinnage pour le renforcement de la connexité



%========== Constantes ==========%

Surf = sqrt(hauteur*largeur/k); % Distance initiale entre les centres
rn = fix(n/2); % "Rayon" du voisinnage pour l'affinement de la grille


%========================================================%
%========== Placement et affinement des germes ==========%
%========================================================%
tic

% Création des matrices de coordonnées pour les centres des superpixels
[indCentresLigne, indCentresColonne] = meshgrid(round((Surf/2):Surf:hauteur), round((Surf/2):Surf:largeur));

nbrsuperPixel = numel(indCentresLigne); % Nombre total de superpixels

% Initialisation de la matrice des centres
centres = zeros(nbrsuperPixel, 5);

for k = 1:nbrsuperPixel
    % Indices de lignes et de colonnes des pixels du voisinage
    row_indices = indCentresLigne(k)-rn : indCentresLigne(k)+rn;
    col_indices = indCentresColonne(k)-rn : indCentresColonne(k)+rn;
    
    % Creation de la sous matrice du voisinnage
    smat = mat(row_indices, col_indices, :);
    
    % Calcul des gradients du voisinnage
    gradient_magnitude_rgb = sqrt(imgradient(smat(:,:,1),"prewitt").^2 + ...
                                   imgradient(smat(:,:,2),"prewitt").^2 + ...
                                   imgradient(smat(:,:,3),"prewitt").^2);


    % Attribution du pixel avec le plus faible gradient
    [~, ind] = min(gradient_magnitude_rgb(:));
    [row, col] = ind2sub(size(gradient_magnitude_rgb), ind);
    centres(k, :) = smat(row, col, :);
end

tAffiGrille = toc;



%=============================%
%========== K-means ==========%
%=============================%

exit = true; % Condition d'arret
nbIterSP = 0; % Compteur d'itérations
etiq = zeros(hauteur,largeur); % Matrice des labels
centrevoi = false(nbrsuperPixel,1); % Vecteur de selection des centres

% Initialisation des vecteurs pour stocker les temps de calcul
tCalculSpxl = zeros(nbIterSPmax,1);
tMajCentres = zeros(nbIterSPmax,1);

figure;

while exit && nbIterSP < nbIterSPmax

    %========== Calcul des superpixels ==========%

    tic
    
    % Parcours de chaque pixel de l'image
    for i = 1:hauteur
        voisX1 = max(1,i-Surf);
        voisX2 = min(hauteur,i+Surf);
        for j = 1:largeur
            voisY1 = max(1,j-Surf);
            voisY2 = min(largeur,j+Surf);

            % Vérification de l'appartenance aux centres voisins
            for k = 1:nbrsuperPixel
                centreX = centres(k,4);
                centrevoi(k) = centreX > voisX1 && centreX < voisX2 && centres(k,5) > voisY1 && centres(k,5) < voisY2;
            end

            indcentre = find(centrevoi); % Indices des centres voisins

            % Calcul de la distance entre le point et les centres du voisinnage
            calcul = (permute(mat(i,j,:),[1 3 2]) - centres(centrevoi,:)).^2;
            distColor = sqrt(sum(calcul(:,1:3), 2));
            distXY = sqrt(sum(calcul(:,4:5), 2));
            dist = distColor + (m/Surf) * distXY;
            
            % Attribution de l'étiquette du centre le plus proche
            [~, ind] = min(dist,[],1);
            etiq(i,j) = indcentre(ind);
        end
    end

    tCalculSpxl(nbIterSP + 1) = toc;

    

    %========== Mise à jour des centres ==========%

    tic
    
    % Initialisation des nouvelles coordonnées des centres
    centresNew = zeros(nbrsuperPixel,5);
    nbrCentres = zeros(nbrsuperPixel,1);

    % Parcours de chaque pixel de l'image
    for i=1:hauteur
        for j = 1:largeur
            % Ajout de la valeur du pixel au centre correspondant
            centresNew(etiq(i,j),:) = centresNew(etiq(i,j),:) + permute(mat(i,j,:), [1 3 2]);
            nbrCentres(etiq(i,j)) = nbrCentres(etiq(i,j)) + 1;
        end
    end
    
    % Calcul des nouveaux centres
    centresNew = centresNew ./ nbrCentres;
    
    % Calcul de l'erreur résiduelle
    calcul = (centres - centresNew).^2;
    distColor = sqrt(sum(calcul(:,1:3), 2));
    distXY = sqrt(sum(calcul(:,4:5), 2));         
    E = sum(distColor + (m/Surf) * distXY);

    % Mise à jour des centres
    centres = centresNew;

    % Vérification de la condition d'arrêt
    exit = E >= Seuil;

    nbIterSP = nbIterSP + 1;

    % Affichage des superpixels actuels
    imagesc(etiq);
    colormap(jet);
    title("Superpixels")
    hold on;
    plot(centres(:, 5), centres(:, 4), 'ro', 'MarkerSize', 2);
    hold off;

    drawnow;

    tMajCentres(nbIterSP) = toc;

end

figure;
imagesc(etiq);
colormap(jet);
title("Superpixels")



%==================================================%
%========== Renforcement de la connexité ==========%
%==================================================%

tic

% Initialisation des vecteurs du nombre de composantes connexes pour chaque superpixel
nbrCCnow = zeros(nbrsuperPixel,1);
nbrCCattendu = ones(nbrsuperPixel,1);

nbIterRC = 0;

% Boucle tant qu'il existe des parties disjointes
while nbrCCnow ~= nbrCCattendu

    % Parcours de chaque superpixel
    for k = 1:nbrsuperPixel

        % Extraction des composantes connexes de l'étiquetage actuel
        CC = bwconncomp(etiq == k);

        % Nombre de composantes connexes pour ce superpixel
        nbrCCnow(k) = CC.NumObjects;
        PixelIndList = CC.PixelIdxList;
        
        % Si le nombre de composantes connexes est supérieur à 1
        if nbrCCnow(k) > 1

            % Calcul de la taille de chaque composante connexe
            taille = cellfun(@numel, PixelIndList);
            [~, ind] = max(taille);
            indCCsup = setdiff(1:nbrCCnow(k), ind); % Indice des composantes connexes sans la dominante
            
            % Parcours des composantes connexes
            for i = indCCsup
                % Parcours de chaque pixel de la composante connexe
                for p = PixelIndList{i}
                    [row, col] = ind2sub(size(etiq), p);

                    % Extraction du voisinage autour du pixel
                    setiq = etiq(max(1, row-voisCC):min(hauteur, row+voisCC), max(1, col-voisCC):min(largeur, col+voisCC));
                    
                    % Sélection de l'étiquette la plus fréquente dans le voisinage
                    nbrLabelvoi = histcounts(setiq, 1:nbrsuperPixel+1);
                    nbrLabelvoi(etiq(p)) = 0;
                    [~, ind] = max(nbrLabelvoi);
                    etiq(p) = ind;
                end
            end
        end
    end

    nbIterRC = nbIterRC + 1;

end

tRenfoCnx = toc;

% Affichage des superpixels après le renforcement de la connexité
figure;
imagesc(etiq);
colormap(jet);
title("Superpixels après renforcement de la connexité ")

% Affichage de l'image d'origine avec les contours des superpixels
figure;
imshow(imagetest);
hold on;
for i = 1:nbrsuperPixel
    boundaries = bwboundaries(etiq == i, 8, "noholes");
    for k = 1:length(boundaries)
        boundary = boundaries{k};
        plot(boundary(:,2), boundary(:, 1), 'r', 'LineWidth', 2);
    end
end
hold off;

fprintf('========== Temps de calcul ==========\n');
fprintf('   Placement et affinement des germes : %d\n', tAffiGrille);
fprintf('   Calcul des superpixels : %d\n', mean(nonzeros(tCalculSpxl)));
fprintf('   Mise à jour des centres : %d\n', mean(nonzeros(tMajCentres)));
fprintf('   Renforcement de la connexité : %d\n', mean(nonzeros(tRenfoCnx)));

fprintf("\n========== Nombres d'itérations ==========\n");
fprintf('   Calcul des superpixels : %d (%d)\n', nbIterSP, nbIterSPmax);
fprintf('   Renforcement de la connexité : %d\n', nbIterRC);

fprintf("\n========== Erreur résiduelle ==========\n");
fprintf('   E = %d\n', E);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A COMPLETER                                             %
% Binarisation de l'image à partir des superpixels        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Sélection des superpixels "jaunes lumineux"
LumiJaune = centres(:,3) > 0 & centres(:,1) > 10;

% Création de masque
Binaire = ismember(etiq,find(LumiJaune));

% Affichage du masque
figure;
imshow(Binaire);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A FAIRE SI VOUS UTILISEZ LES MASQUES BINAIRES FOURNIS   %
% Chargement des masques binaires                         %
% de taille nb_lignes x nb_colonnes x nb_images           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ... 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A DECOMMENTER ET COMPLETER                              %
% quand vous aurez les images segmentées                  %
% Affichage des masques associes                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure;
% subplot(2,2,1); ... ; title('Masque image 1');
% subplot(2,2,2); ... ; title('Masque image 9');
% subplot(2,2,3); ... ; title('Masque image 17');
% subplot(2,2,4); ... ; title('Masque image 25');

% chargement des points 2D suivis 
% pts de taille nb_points x (2 x nb_images)
% sur chaque ligne de pts 
% tous les appariements possibles pour un point 3D donne
% on affiche les coordonnees (xi,yi) de Pi dans les colonnes 2i-1 et 2i
% tout le reste vaut -1
pts = load('viff.xy');
% Chargement des matrices de projection
% Chaque P{i} contient la matrice de projection associee a l'image i 
% RAPPEL : P{i} est de taille 3 x 4
load dino_Ps;

% Reconstruction des points 3D
X = []; % Contient les coordonnees des points en 3D
color = []; % Contient la couleur associee
% Pour chaque couple de points apparies
for i = 1:size(pts,1)
    % Recuperation des ensembles de points apparies
    l = find(pts(i,1:2:end)~=-1);
    % Verification qu'il existe bien des points apparies dans cette image
    if size(l,2) > 1 & max(l)-min(l) > 1 & max(l)-min(l) < 36
        A = [];
        R = 0;
        G = 0;
        B = 0;
        % Pour chaque point recupere, calcul des coordonnees en 3D
        for j = l
            A = [A;P{j}(1,:)-pts(i,(j-1)*2+1)*P{j}(3,:);
            P{j}(2,:)-pts(i,(j-1)*2+2)*P{j}(3,:)];
            R = R + double(im(int16(pts(i,(j-1)*2+1)),int16(pts(i,(j-1)*2+2)),1,j));
            G = G + double(im(int16(pts(i,(j-1)*2+1)),int16(pts(i,(j-1)*2+2)),2,j));
            B = B + double(im(int16(pts(i,(j-1)*2+1)),int16(pts(i,(j-1)*2+2)),3,j));
        end;
        [U,S,V] = svd(A);
        X = [X V(:,end)/V(end,end)];
        color = [color [R/size(l,2);G/size(l,2);B/size(l,2)]];
    end;
end;
fprintf('Calcul des points 3D termine : %d points trouves. \n',size(X,2));

%affichage du nuage de points 3D
figure;
hold on;
for i = 1:size(X,2)
    plot3(X(1,i),X(2,i),X(3,i),'.','col',color(:,i)/255);
end;
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A COMPLETER                  %
% Tetraedrisation de Delaunay  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T = ...                      

% A DECOMMENTER POUR AFFICHER LE MAILLAGE
% fprintf('Tetraedrisation terminee : %d tetraedres trouves. \n',size(T,1));
% Affichage de la tetraedrisation de Delaunay
% figure;
% tetramesh(T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A DECOMMENTER ET A COMPLETER %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calcul des barycentres de chacun des tetraedres
% poids = ... 
% nb_barycentres = ... 
% for i = 1:size(T,1)
    % Calcul des barycentres differents en fonction des poids differents
    % En commencant par le barycentre avec poids uniformes
%     C_g(:,i,1)=[ ...

% A DECOMMENTER POUR VERIFICATION 
% A RE-COMMENTER UNE FOIS LA VERIFICATION FAITE
% Visualisation pour vérifier le bon calcul des barycentres
% for i = 1:nb_images
%    for k = 1:nb_barycentres
%        o = P{i}*C_g(:,:,k);
%        o = o./repmat(o(3,:),3,1);
%        imshow(im_mask(:,:,i));
%        hold on;
%        plot(o(2,:),o(1,:),'rx');
%        pause;
%        close;
%    end
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A DECOMMENTER ET A COMPLETER %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copie de la triangulation pour pouvoir supprimer des tetraedres
% tri=T.Triangulation;
% Retrait des tetraedres dont au moins un des barycentres 
% ne se trouvent pas dans au moins un des masques des images de travail
% Pour chaque barycentre
% for k=1:nb_barycentres
% ...

% A DECOMMENTER POUR AFFICHER LE MAILLAGE RESULTAT
% Affichage des tetraedres restants
% fprintf('Retrait des tetraedres exterieurs a la forme 3D termine : %d tetraedres restants. \n',size(Tbis,1));
% figure;
% trisurf(tri,X(1,:),X(2,:),X(3,:));

% Sauvegarde des donnees
% save donnees;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONSEIL : A METTRE DANS UN AUTRE SCRIPT %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load donnees;
% Calcul des faces du maillage à garder
% FACES = ...;
% ...

% fprintf('Calcul du maillage final termine : %d faces. \n',size(FACES,1));

% Affichage du maillage final
% figure;
% hold on
% for i = 1:size(FACES,1)
%    plot3([X(1,FACES(i,1)) X(1,FACES(i,2))],[X(2,FACES(i,1)) X(2,FACES(i,2))],[X(3,FACES(i,1)) X(3,FACES(i,2))],'r');
%    plot3([X(1,FACES(i,1)) X(1,FACES(i,3))],[X(2,FACES(i,1)) X(2,FACES(i,3))],[X(3,FACES(i,1)) X(3,FACES(i,3))],'r');
%    plot3([X(1,FACES(i,3)) X(1,FACES(i,2))],[X(2,FACES(i,3)) X(2,FACES(i,2))],[X(3,FACES(i,3)) X(3,FACES(i,2))],'r');
% end;
