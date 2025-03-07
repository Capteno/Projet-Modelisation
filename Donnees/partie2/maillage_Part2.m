close all;
clear all;
addpath("functions/");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Maillage de la surface. %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load T
load indicesBonsBarycentres.mat
load X

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Tri %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

tri= T.Triangulation;
tri = tri(indicesBonsBarycentres,:);

% Affichage
figure;
trisurf(tri,X(1,:),X(2,:),X(3,:));

% Nombre de tétraèdre
nbr_tetra = size(tri,1);

% Création de la matrice

% Calcul des faces du maillage à garder
FACES = zeros(nbr_tetra*4,3);

% compléter le vecteur FACES
for i = 1:(nbr_tetra)
    i0 = (i-1)*4 + 1; % Modifier l'index de départ
    i1 = i*4;
    FACES(i0:i1,:) = nchoosek(tri(i,:),3);
end

% Trie des faces
FACES = sortrows(FACES);

% On enlève les occurrences
i = 1;
while i < size(FACES, 1)
    if isequal(FACES(i,:), FACES(i+1,:))
        FACES(i+1,:) = [];
    else
        i = i + 1; %
    end
end

fprintf('Calcul du maillage final termine : %d faces. \n',size(FACES,1));

%Affichage du maillage final
figure;
hold on
nbr_faces = size(FACES,1);
tic
for i = 1:nbr_faces
   hold on
   plot3([X(1,FACES(i,1)) X(1,FACES(i,2))],[X(2,FACES(i,1)) X(2,FACES(i,2))],[X(3,FACES(i,1)) X(3,FACES(i,2))],'r');
   plot3([X(1,FACES(i,1)) X(1,FACES(i,3))],[X(2,FACES(i,1)) X(2,FACES(i,3))],[X(3,FACES(i,1)) X(3,FACES(i,3))],'r');
   plot3([X(1,FACES(i,3)) X(1,FACES(i,2))],[X(2,FACES(i,3)) X(2,FACES(i,2))],[X(3,FACES(i,3)) X(3,FACES(i,2))],'r');
end;

toc
