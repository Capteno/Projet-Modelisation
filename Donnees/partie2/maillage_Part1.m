close all;
clear all;
addpath("functions/");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Triangulation. %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = triangulationCode();
save('X.mat','X');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tetraedrisation de Delaunay  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = DelaunayTri(X(1,:)',X(2,:)',X(3,:)');                

% A DECOMMENTER POUR AFFICHER LE MAILLAGE
fprintf('Tetraedrisation terminee : %d tetraedres trouves. \n',size(T,1));
%Affichage de la tetraedrisation de Delaunay
%figure;
%tetramesh(T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calcul des barycentres. %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C_g = barycentres(T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Filtrage des tétraèdres %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[indicesMauvaisBarycentres] = triTetraedres(C_g);

% Récupérer les indices qui ne sont pas dans indicesMauvaisBarycentres
indicesBonsBarycentres = setdiff(1:size(C_g, 1), indicesMauvaisBarycentres);

% Conserver uniquement les barycentres valides
C_g = C_g(indicesBonsBarycentres, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Verification des barycentres %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%verificationBarycentres(C_g,im_mask,1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Tri des tétraèdres %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copie de la triangulation pour pouvoir supprimer des tetraedres
tri= T.Triangulation;
tri = tri(indicesBonsBarycentres,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Affichage des tétraèdres %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% A DECOMMENTER POUR AFFICHER LE MAILLAGE RESULTAT
% Affichage des tetraedres restants
% fprintf('Retrait des tetraedres exterieurs a la forme 3D termine : %d tetraedres restants. \n',size(Tbis,1));
figure;
trisurf(tri,X(1,:),X(2,:),X(3,:));

% Sauvegarde des donnees
save('T.mat','T');
save('indicesBonsBarycentres.mat','indicesBonsBarycentres');