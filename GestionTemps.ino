//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waGestionTemps, 64);

// Declare thread function for GestionTemps (priority 32).
NIL_THREAD(GestionTemps, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeGestionTemps);  
    
    //Algo
    static long tps_debut_match = 0;
    static long tmp_prec = micros();
    long tmp_actu;
    if(MATCH_EN_COURS)
    {
      if(tps_debut_match == 0)
        tps_debut_match = micros();
      
      tmp_actu = micros() - tps_debut_match;
      
      if(tmp_actu > 90*1000000)
      {
        Serial.println("Fin de match");
        FIN_MATCH = true;
        MATCH_EN_COURS = false;
      }   
      else
      { 
        if(tmp_actu/1000000 != tmp_prec)
       { 
        /*  Serial.println(tmp_actu/1000000);*/
          tmp_prec = tmp_actu;
  /*  Serial.print(geneTraj.dist.consigneFiltree);
    Serial.println("||");
    Serial.println(geneTraj.angle.consigneFiltree);*/
       } 
      }   
    }
    
      /* if(MATCH_EN_COURS)
            {  
                if(bouclePosition.sensDeplacement == AVANCE)
                {
                  mG.mot(1000*coeffObstacle);
                  mD.mot(1000.0*coeffObstacle);
                }
                else
                {
                  mG.mot(-1000.0*coeffObstacle);
                  mD.mot(-1000.0*coeffObstacle);
                }
            }
            else
              mG.mot(0);
              mD.mot(0);    
    */
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print("-");  
    #endif
    
    //Appel de la tache d'intelligence
    nilSemSignal(&wakeIntelligence);  
  }  
}
