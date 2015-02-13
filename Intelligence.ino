//------------------------------------------------------------------------------
#define PROGRAMME_HOMOLOGATION
//#define PROGRAMME_MATCH

// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waIntelligence, 64);
int i=0; //cmpteur

// Declare thread function for Intelligence (priority 33).
NIL_THREAD(Intelligence, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeIntelligence);  
    
    //Algo
    if(!FIN_MATCH && !MATCH_EN_COURS && 1) //Triche, ne fonctionne jamais
    {
      if(digitalRead(PIN_JACK_DEPART) == 1)
      {
        Serial.println("Debut du match!!");
        
        MATCH_EN_COURS = true;
      }
    }
    
    if(MATCH_EN_COURS)
    {
      /* BIFFLE
         geneTraj.dist.consigneFiltree = 2160;
         geneTraj.angle.consigneFiltree = 0;
         nilThdSleepMilliseconds(1000);
         
         geneTraj.dist.consigneFiltree = 2160; //50cm
         geneTraj.angle.consigneFiltree = -constPI/2;
         nilThdSleepMilliseconds(500);     
         geneTraj.dist.consigneFiltree = 2160;
         geneTraj.angle.consigneFiltree = constPI/2;
         nilThdSleepMilliseconds(1000);     
         geneTraj.dist.consigneFiltree = 2160;
         geneTraj.angle.consigneFiltree = -constPI/2;
         nilThdSleepMilliseconds(1000);     
         geneTraj.dist.consigneFiltree = 2160;
         geneTraj.angle.consigneFiltree = 0;
         nilThdSleepMilliseconds(500);     
      while(1);
      */
      
#ifdef PROGRAMME_MATCH
  
if(CouleurDepart == BLEU)
{
  //Avance vers cadeau
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = 0;
         nilThdSleepMilliseconds(1000);
         
  //Rotation vers cadeau
      // on enleve la détection
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = false;         
      // tour de 90° vers la droite
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(1000);     
         
  //Avance pour chute du cadeau
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = false;
         
         geneTraj.dist.consigneFiltree = 2530+1300;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(1000);     
         
  //Recul pour position initiale 
         bouclePosition.sensDeplacement = RECULE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(1000);    
         
 
   //Rotation pour position initiale
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = constPI/4;
         nilThdSleepMilliseconds(1000);      
      
   //Fin de match
      while (1)
       {
          nilThdSleepMilliseconds(2000);  
       }
}
else
{
  //Avance vers cadeau
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = 0;
         nilThdSleepMilliseconds(1000);
         
  //Rotation vers cadeau
      // on enleve la détection
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = false;         
      // tour de 90° vers la gauche
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = +constPI/1.4;
         nilThdSleepMilliseconds(1000);     
         
  //Avance pour chute du cadeau
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = false;
         
         geneTraj.dist.consigneFiltree = 2530+1300;
         geneTraj.angle.consigneFiltree = +constPI/1.4;
         nilThdSleepMilliseconds(1000);     
         
  //Recul pour position initiale 
         bouclePosition.sensDeplacement = RECULE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = +constPI/1.4;
         nilThdSleepMilliseconds(1000);    
         
 
   //Rotation pour position initiale
         bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = -constPI/4;
         nilThdSleepMilliseconds(1000);      
      
   //Fin de match
      while (1)
       {
          nilThdSleepMilliseconds(2000);  
       }    
}   
#else
//prog homologation
    #ifdef PROGRAMME_HOMOLOGATION
        /**  bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;*/
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = 0;
         nilThdSleepMilliseconds(10000);
         
      // on enleve la détection
          detectionObstacle = false;
          
      // tour de 90° vers la droite
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(10000);     
         
         
      /*   bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = false;*/
         
         geneTraj.dist.consigneFiltree = 2530+1300;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(2000);     
         
         bouclePosition.sensDeplacement = RECULE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = -constPI/1.4;
         nilThdSleepMilliseconds(2000);    
         
 
         
         
         geneTraj.dist.consigneFiltree = 2530;
         geneTraj.angle.consigneFiltree = constPI/4;
         nilThdSleepMilliseconds(2000);      

       bouclePosition.sensDeplacement = AVANCE;
         detectionObstacle = true;
         
         geneTraj.dist.consigneFiltree = 2530+2000;
         geneTraj.angle.consigneFiltree = constPI/4;
         nilThdSleepMilliseconds(2000);  
        

        detectionObstacle=false;
      while (i<100)
       {
       /* Demi tour arrivé en bout de table */
        
       geneTraj.dist.consigneFiltree = 2530+2000;
       geneTraj.angle.consigneFiltree = -constPI/4;
       nilThdSleepMilliseconds(1000);   
       
       geneTraj.dist.consigneFiltree = 2530+2000;
       geneTraj.angle.consigneFiltree = +constPI/4;
       nilThdSleepMilliseconds(1000);   
       }
       
          
    #endif
#endif

         while(1);
    }
    else
    {
         geneTraj.dist.consigneFiltree = 0;
         geneTraj.angle.consigneFiltree = 0;
    }
    

    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.println("/\\");  
    #endif
  }  
}
