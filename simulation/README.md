# Simulateur

L'environnement actuel simule :
- résistance aérodynamique

- résistance au roulement

- résistance de pente

- moteur limité en force et en puissance

- transmission

- freinage

- cinématique correcte

- puissance aux roues

- consommation carburant cumulée

- affichage graphique multi-canal

Les prochains points à ajoutés afin de rendre le simulateur plus réaliste :
- courbes et direction 
- boîtes de vitesses réelles
- limites d'adhérence
- limites de puissance moteurs (courbe couple/ régime )
- freinage réel 
- BSFC variable en fonction du couple et du régime 
- inertie de rotation des pneus + moteur
- température moteur 
- gestions batterie, ou capacité réservoir
- vent pluie, densité de l'air


## Ajout de NEAT 

L'ajout de NEAT afin d'explorer, améliorer la performance d'un moteur de voiture. Il trouvera une configuration qui maximise un score de performance définis.

Il va recevoir un ensemble de paramètres du moteur, de la transmission ou du comportement.
L'environnement simule un parcours. 
Le score retourne performance + consommation
Neat sélection donc les meilleurs, mutent et recommence. 
A la fin on obtient une configuration optimale pour un moteur de voiture.
