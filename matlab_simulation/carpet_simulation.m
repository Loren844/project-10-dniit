% Test avec trois objet en 3D. On simule le mouvement de cubes en 3D dans
% l'espace, et on créé notre caméra. Elle n'est plus un simple élément du 
% décort car on passe en vue à la 1ère perssone

%%%%%%% 1 - Définition des paramètres (issus de la documentation) %%%%%%%

% focal de la caméra, déffinit le zoom naturel de la caméra (unité : pixel)
fx = 593.23801979; % grandissement horizontal
fy = 593.70051809; % grandissement vertical

% centre optique de la caméra, à cause des défaut de fabrications ce n'est
% pas toujours le centre de l'image, décaler ce centre revient à décaler
% toute l'image, c'est pour cela que le bras a besoin de conaitre le
% décalage naturel au départ. Exprimé en pixel, on voit qu'il y as un
% décalage important par raport à (320, 240)
cx = 167.73051341;
cy = 265.51031581;

% Résolution de l'image (ex: 640x480)
% convention (lignes,colonnes) donc (hauteur, largeur)
imageSize = [480, 640]; 

% Créé la matrice intrinsec K de notre caméra. Le type de retour est un 
% objet de type cameraIntrinsics qui possède des propriétés comme 
% camera.IntrinsicMatrix ou camera.FocalLength
camera = cameraIntrinsics([fx fy], [cx cy], imageSize);


%%%%%%%%%%%%%%%% 2 - Création des objets virtuels en 3D %%%%%%%%%%%%%%%%

% Création des cubes, une colisionBox c'est à la fois la boite de colision
% et l'objet 3D visible.L'objet de type colisionBox possède une géométrie 
% fixe (la forme de l'objet) et une propriété .Pose (sa postion dans l'espace)
cubeSize = 0.1; 
monCube1 = collisionBox(cubeSize, cubeSize, cubeSize);
monCube2 = collisionBox(cubeSize, cubeSize, cubeSize);
monCube3 = collisionBox(cubeSize, cubeSize, cubeSize);

% 2. Définition des coordonnées de base
coordsCube1 = [0, 0, 0.1];
coordsCube2 = [-0.3, -0.2, 0.1];
coordsCube3 = [-0.6, 0.2, 0.1]; 

% 3. Conversion du triplé en une matrice de "Pose" 4x4
% trvec2tform = "Translation Vector to Transformation Matrice"
% la matrice de pose donne à la fois la position et l'oriantation.
monCube1.Pose = trvec2tform(coordsCube1);
monCube2.Pose = trvec2tform(coordsCube2);
monCube3.Pose = trvec2tform(coordsCube3);


%%%%%%%%%%%%%%%%% 2 - Création du tapis virtuel en 3D %%%%%%%%%%%%%%%%%

% Création de la forme (le volume reste centré sur [0,0,0] par défaut)
cubeSize = 0.1; 
monTapis = collisionBox(40*cubeSize, 5*cubeSize, cubeSize);

% 2. Définition des coordonnées de base
coordsTapis = [1, 0, 0]; 

% 3. Conversion du triplé en une matrice de "Pose" 4x4
% trvec2tform = "Translation Vector to Transformation Matrice"
% la matrice de pose donne à la fois la position et l'oriantation
monTapis.Pose = trvec2tform(coordsTapis);


%%%%%%%%%%%%%% 3 - Création et positionement de la caméra %%%%%%%%%%%%%%

% On positione la caméra sur le coté du tapis, en hauteur
camPos = [1, -1.9, 1]; 

% Rotation : regarde vers les Y négatifs, et un peu vers le bas
rotation_principale = axang2tform([1 0 0, 3*pi/2]);
inclinaison_bas = axang2tform([1 0 0, -0.3]); % environ 17 degrés
rotation_gauche = axang2tform([0 0 1, 0.4]); % Environ 23 degrés vers la gauche

% Créer la matrice de pose finale, sert au calcul mais ne gère pas l'affichage
cameraPose = trvec2tform(camPos)*rotation_principale*inclinaison_bas*rotation_gauche;


%%%%%%%%%%%%%% 4 - Simulation du mouvement de l'objet créé %%%%%%%%%%%%%%

% ouverture d'une nouvelle fenetre, sans cela MathLab pourait dessiner par
% dessus une fenetre déjà ouverte.
figure;

% Affiche les cube dans un espace 3D
ax = show(monCube1);
show(monCube2, 'Parent', ax);
show(monCube3, 'Parent', ax);

% Affiche le tapis dans l'espace 3D
show(monTapis, 'Parent', ax);

% dit à MATLAB de conserver les points précédents sur le graphique. 
% Sans elle, chaque nouveau point effacerait le précédent, et 
% on ne verrais pas la trajectoire.
hold on;

% Affiche une grille de lecture.
grid on;

% Force la vue en 3D (perspective)
view(3); 

% force les axes à être proportionels, sinon notre cube est defformé
axis equal;

% On fixe les limites pour ne pas qu'il y ait un dé-zoom continu
axis([-1 3 -2 1 -1 2]); 

%%%% on affiche la caméra et on passe en 1ère perssone. %%%%%

% Dessiner le repère d'orientation (Axes X, Y, Z de la caméra)
% Cela remplace le triangle plat par un repère 3D réel
plotTransforms(camPos, tform2quat(cameraPose), 'FrameSize', 0.15);

% affiche le texte "objectif caméra à 0,1 mètres au dessus
text(camPos(1), camPos(2), camPos(3) + 0.1, ' Objectif Caméra');

% 2. Configurer la vue "Première Personne" (L'œil de MATLAB)
% On place l'œil exactement à la position définie dans cameraPose
campos(ax, tform2trvec(cameraPose));
% On définit la cible du regard en utilisant la direction Z de la matrice
% (La caméra regarde toujours vers son propre axe Z local)
directionRegard = tform2rotm(cameraPose) * [0; 0; 1]; 
camtarget(ax, tform2trvec(cameraPose) + directionRegard');

% 3. Paramètres optiques pour simuler la lentille
camproj(ax, 'perspective'); % Active la déformation de distance
camva(ax, 45); % Angle d'ouverture (Field of View)

%%%%%%%%%%%%%% fin de l'affichage de la caméra %%%%%%%%%%%%%

% 3. Animation
for t = 1:200
    % on suprime les ancients cubes
    delete(findobj(ax, 'Type', 'patch'));

    % On déplace les cubes physiquement
    newPos1 = [t*0.01, 0, 0.1]; 
    monCube1.Pose = trvec2tform(newPos1);
    
    newPos2 = [-0.3 + t*0.01, -0.2, 0.1]; 
    monCube2.Pose = trvec2tform(newPos2);
    
    newPos3 = [-0.6 + t*0.01, 0.2, 0.1]; 
    monCube3.Pose = trvec2tform(newPos3);
    
    % On met à jour l'affichage 3D
    show(monCube1, 'Parent', ax);
    show(monCube2, 'Parent', ax);
    show(monCube3, 'Parent', ax);
    show(monTapis, 'Parent', ax);
    drawnow;
    
    % marque une pause de 0,1 sec pour qu'on distingue le mouvement
    pause(0.0001);
end