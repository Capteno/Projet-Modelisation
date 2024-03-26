function C_g = barycentres(T)

% Initialisation de la variable
C_g = [];

for i = 1:size(T,1)

    % Récupération des points du tétraèdre courant
    p = T.Triangulation(i,:);

    % Récupération des points
    barycentre = [T.X(p(1),:);
        T.X(p(2),:);
        T.X(p(3),:);
        T.X(p(4),:)];

    % Calcul des barycentres
    barycentre = mean(barycentre,1);

    % Concaténation avec 1 pour avoir un point homogène
    barycentre = [barycentre 1];

    % Ajout dans la variable des barycentres
    C_g= [C_g;barycentre];
end

