#ifndef _DRIVER_MOTEUR_HEADER_
#define _DRIVER_MOTEUR_HEADER_

//Includes
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
  
//Constantes
  //Mode de fonctionnement
  #define modBrake  0    //Petit frein sur l'axe moteur
  #define modCW     1    //Clockwise (sens horaire) | OUT1 = 0, OUT2 = PWM
  #define modCCW    2    //Counter Clockwise (sens antihoraire) | OUT1 = PWM, OUT2 = 0
  #define modStop   3    //Sorties deconnectee
  //NB : un mode normal (tout sauf modStop) avec une PWM = 0 amene aussi un "petit frein"
  
//Classe
  class DriverMoteur
  {
    public:
    DriverMoteur(byte dir, byte brake, byte pwm, byte isens = 255, byte therm = 255);
    void mot(int vit);
    byte getTherm();
    byte getISens();

    private:
    void motMode(byte mode);
    void motMap(byte dir, byte brake, byte pwm, byte isens = 255, byte therm = 255);

    //Variables
      //Les numeros de pin de chaque signaux de commande du driver
      byte _DIR, _BRAKE, _PWM, _ISENS, _THERM;
      boolean _RETOUR_INFO;   //True si _ISENS et _THERM déclarés
  };
  
#endif
