//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waOrdonnanceur, 64);

// Declare thread function for Ordonnanceur (priority 41).
NIL_THREAD(Ordonnanceur, arg) {  
  /*Environnement*/
  unsigned int n = 0;
  
  nilTimer1Start(5000);  
  
  /*Algo*/
    
  //Affichage Ordonnancement
  #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
  uint32_t last = micros();
  #endif
  // Execute while loop every 200ms seconds.
  while (TRUE)
  //nilThdSleep(1000);  //Stop l'ordonnancement
  {
    for(int GeneTraj = 1; GeneTraj <= 10; GeneTraj++)  // 10 * 20ms = 200ms
    {
      for(int odo = 1; odo <= 4; odo++)  // 4 * 5ms = 20ms
      {
        //Appel de la tache d'odométrie
        nilSemSignal(&wakeOdometrie);
        nilTimer1Wait();  //Période de 5ms
      }
      //Appel de la tache de génération de trajectoires
      nilSemSignal(&wakeGenerateurTrajectoire);
    }   
    //Appel de la tache de gestion des capteurs
    nilSemSignal(&wakeCapteur);
    
    
    //Affichage standard
    #if (AFFICHAGE == ON && AFFICHAGE_STANDARD == ON)
    Serial.print("Obstacles : ");
    Serial.print(dist.fg);
    Serial.print(' ');
    Serial.print(dist.fd);
    Serial.print(' ');
    Serial.print(dist.rg);
    Serial.print(' ');
    Serial.println(dist.rd);
    
    Serial.print("Encodeurs -> G: ");
    Serial.print(pos.encG);
    Serial.print(" D: ");
    Serial.println(pos.encD);
    
    Serial.print("Position -> X: ");
    Serial.print(pos.x);
    Serial.print(" Y: ");
    Serial.print(pos.y);
    Serial.print(" A: ");
    Serial.println(pos.a);    
    #endif
    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    uint32_t t = micros();
    Serial.print(t - last);
    Serial.print(' ');
    Serial.println(n++);    
    last = t;
    #endif
  }
}


//------------------------------------------------------------------------------
/*
 * Threads static table, one entry per thread.  A thread's priority is
 * determined by its position in the table with highest priority first.
 *
 * These threads start with a null argument.  A thread's name is also
 * null to save RAM since the name is currently not used.
 */
NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY(NULL, Odometrie, NULL, waOdometrie, sizeof(waOdometrie)) //Priority Level : 11
NIL_THREADS_TABLE_ENTRY(NULL, Asservissement, NULL, waAsservissement, sizeof(waAsservissement))  //Priority Level : 12
NIL_THREADS_TABLE_ENTRY(NULL, GenerateurTrajectoire, NULL, waGenerateurTrajectoire, sizeof(waGenerateurTrajectoire))  //Priority Level : 21
NIL_THREADS_TABLE_ENTRY(NULL, Capteur, NULL, waCapteur, sizeof(waCapteur))  //Priority Level : 31
NIL_THREADS_TABLE_ENTRY(NULL, Intelligence, NULL, waIntelligence, sizeof(waIntelligence))  //Priority Level : 33
NIL_THREADS_TABLE_ENTRY(NULL, GestionTemps, NULL, waGestionTemps, sizeof(waGestionTemps))  //Priority Level : 32
NIL_THREADS_TABLE_ENTRY(NULL, BouclePosition, NULL, waBouclePosition, sizeof(waBouclePosition))  //Priority Level : 34
NIL_THREADS_TABLE_ENTRY(NULL, Ordonnanceur, NULL, waOrdonnanceur, sizeof(waOrdonnanceur))  //Priority Level : 41
NIL_THREADS_TABLE_END()


//------------------------------------------------------------------------------
// Loop is the idle thread.  The idle thread must not invoke any
// kernel primitive able to change its state to not runnable.
void loop() {
  #if (AFFICHAGE == ON && AFFICHAGE_STACKUSE == ON)
  nilSysLock();  
  Serial.println();
  Serial.println();
  nilPrintStackSizes(&Serial);
  nilPrintUnusedStack(&Serial);
  Serial.println();
  Serial.println();
  nilSysUnlock();
  #endif
  
  /*Asserv*/
  nilSysLock();  /*
  Serial.print("precD | D : ");
  Serial.print(pos.prec.encD);
  Serial.print(" | ");
  Serial.println(pos.encD);
  Serial.print("E | M | C : ");
  Serial.println(asserv.dist.ecart);
  Serial.print(" | ");
  Serial.print(asserv.dist.variation);
  Serial.println();
  Serial.print(asserv.commande.abs.motG);
  Serial.print(' ');
  Serial.println(asserv.commande.abs.motD);
  Serial.println();
  Serial.print(asserv.commande.motG);
  Serial.print(' ');
  Serial.println(asserv.commande.motD);
  nilSysUnlock();

  
  /*Standard*/
  nilSysLock(); /*
    Serial.print("Obstacles : ");
    Serial.print(dist.fg);
    Serial.print(' ');
    Serial.print(dist.fd);
    Serial.print(' ');
    Serial.print(dist.rg);
    Serial.print(' ');
    Serial.println(dist.rd);
    
    Serial.print("Encodeurs -> G: ");
    Serial.print(pos.encG);
    Serial.print(" D: ");
    Serial.println(pos.encD);
    
    Serial.print("Position -> X: ");
    Serial.print(pos.x);
    Serial.print(" Y: ");
    Serial.print(pos.y);
    Serial.print(" A: ");
    Serial.println(pos.a);    
  nilSysUnlock();*//*
    Serial.print(asserv.commande.motG);
    Serial.print(' ');
    Serial.println(asserv.commande.motD);*/
    /*
    Serial.println(coeffObstacle);

    Serial.print(geneTraj.dist.consigneFiltree);
    Serial.println("||");
    Serial.println(geneTraj.angle.consigneFiltree);*/
  
    CouleurDepart = digitalRead(PIN_INT_ROUGE);


  // Delay for one second.
  // Must not sleep in loop so use nilThdDelayMilliseconds().
  // Arduino delay() can also be used in loop().
  nilThdDelayMilliseconds(300); //1000
}
