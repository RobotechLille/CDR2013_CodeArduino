/* 
 * L'objectif est de creer une librairie capable de piloter
 * le IC DC Motor Driver LMD18200T facilement
 */

//Includes
#include "DriverMoteur.h"


//Constructeur
DriverMoteur::DriverMoteur(byte dir, byte brake, byte pwm, byte isens, byte therm)
{
  //Mapping des ports
  motMap(dir,brake,pwm,isens,therm);

}


/*
 * Controle complet du moteur
 * Recoit une vitesse positive ou negative
 * (pour la marche arriere) exprimee en pourmille (vitMax = 1000, vitMin = -1000)
 *
 * A une vitesse nulle, le moteur est libre (pas de freinage)
 */
void DriverMoteur::mot(int vit)
{
  if(vit > 0)
  {
    if(vit>1000)vit=1000;     //Saturation à 1000
    motMode(modCW);           //Marche avant
    analogWrite(_PWM,(int)((long)vit*255/1000));
  }
  else if(vit < 0)
  {
  if(vit<-1000)vit=-1000;     //Saturation à -1000
    motMode(modCCW);          //Marche arrière
    analogWrite(_PWM,-(int)((long)vit*255/1000));
  }
  else
    motMode(modStop);  //Laisse l'axe moteur libre, moins risque pour le moteur
}


//Controle les modes du moteur A (Frein, sens horaire, anti-horaire, libre)
void DriverMoteur::motMode(byte mode)
{
  switch(mode)
  {
    case modBrake :
      digitalWrite(_BRAKE,HIGH);
      digitalWrite(_DIR,LOW);
      analogWrite(_PWM,255);
      break;
      
    case modCW :
      digitalWrite(_DIR,HIGH);
      digitalWrite(_BRAKE,LOW);
      break;
   
   case modCCW :
      digitalWrite(_DIR,LOW);
      digitalWrite(_BRAKE,LOW);
     break;
     
   case modStop :
      digitalWrite(_BRAKE,HIGH);
      analogWrite(_PWM,0);
     break;
     
   default : 
      digitalWrite(_BRAKE,HIGH);
      analogWrite(_PWM,0);
     break; 
  }
}

//Modifie le mapping du moteur (indiquer les numeros de pin)
void DriverMoteur::motMap(byte dir, byte brake, byte pwm, byte isens, byte therm)
{ 
  _DIR = dir;
  _BRAKE = brake;
  _PWM = pwm;  
  
  pinMode(_DIR,OUTPUT);
  pinMode(_BRAKE,OUTPUT);
  pinMode(_PWM,OUTPUT);

  if(therm != 255)
  {
    _RETOUR_INFO = true;

    _ISENS = isens;
    _THERM = therm;

    pinMode(_ISENS,INPUT);
    pinMode(_THERM,INPUT);
  }
  else
    _RETOUR_INFO = false;
}


byte DriverMoteur::getTherm()
{
    if (_RETOUR_INFO)
    {
      return digitalRead(_THERM);
    }
    else
      return -1;
}


byte DriverMoteur::getISens()
{
    if (_RETOUR_INFO)
    {
      return digitalRead(_ISENS);
    }
    else
      return -1;
}