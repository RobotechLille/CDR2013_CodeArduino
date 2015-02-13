/* 
 * Nous allons tester une librairie capable de piloter
 * le IC DC Motor Driver LMD18200T facilement
 */

//Includes
#include <DriverMoteur.h>

//Creation d'un objet de classe driverMoteur pour piloter le driver
// Genere le mapping : 
// Pin : Signal
// 30 : DIR
// 31 : BRAKE
// 2  : PWM
DriverMoteur driv(30,31,2);

//Initialisation
void setup()
{
}



//Boucle porgramme
void loop()
{
  //Moteur en marche avant 50% vitesse 3secondes
  driv.mot(500);
  delay(3000);
  driv.mot(0);
  
  //Moteur en marche arriere 70% vitesse 5secondes
  driv.mot(-700);
  delay(5000);
  driv.mot(0);
}

