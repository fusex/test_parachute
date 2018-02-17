# test_parachute

Simple mesure des 10 degrés de liberté du MPU9250 et stockage sur une carte SD.
Chronologie:
- La DEL s'allume
- Ouverture du fichier
- 10 secondes de mesures. Acceleration 3 axes, Gyro 3 axes, Magnéto 3 axes, Altitude.
- Fermeture du fichier
- Extinction de la DEL
- 10 secondes d'attente nouvelle boucle.
