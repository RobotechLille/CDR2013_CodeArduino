//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waCapteur, 64);

// Declare thread function for Capteur (priority 31).
NIL_THREAD(Capteur, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeCapteur);  
    
    //Algo
    //Actualisation des distances capteurs
    dist.fd = usFD.getDistanceCentimeter();
    dist.fg = usFG.getDistanceCentimeter();
    dist.rd = usRD.getDistanceCentimeter();
    dist.rg = usRG.getDistanceCentimeter();
    
    long fd = dist.fd;
    if (fd == -1) fd += 55;
    long fg = dist.fg;
    if (fg == -1) fg += 55;
    long rd = dist.rd;
    if (rd == -1) rd += 55;
    long rg = dist.rg;
    if (rg ==
    -1) rg += 55;
    
    if(detectionObstacle)
    {
      if(bouclePosition.sensDeplacement == AVANCE)
      {
        if(fd < 31 || fg < 31
        )
        {
          if (fd > fg)
            coeffObstacle = (float)(fg - 16)/15;
          else if(fg > fd)
            coeffObstacle = (float)(fd - 16)/15;
            
          if (coeffObstacle < 0) coeffObstacle = 0;
        }
        else
        {
          coeffObstacle =  1;
        }
      }
      else
      {
       if(rd < 20 || rg < 20)
        {
          if (rd > rg)
            coeffObstacle = (float)(rg - 5)/15;
          else if(rg > rd)
            coeffObstacle = (float)(rd - 5)/15;
            
          if (coeffObstacle < 0) coeffObstacle = 0;
        }
        else
        {
          coeffObstacle =  1;
        }
      }
    }
    
    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print("-");  
    #endif
    
    //Appel de la tache de gestion du temps
    nilSemSignal(&wakeGestionTemps);  
  }  
}
