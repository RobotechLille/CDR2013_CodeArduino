/*Reperes************************************************************
*
* Position des axes par rappot au terrain de jeu                      
* ----------------------------------------------
*                                                                                                                                      
*   X--->____________________________________                                
*   |                                        |Y= 2000          
*   |                                        |							              		
*   ^                                        |							           
*  /|\                                       |							              		
*	|                                        |							               
*  Y|___                                  ___|							              	
* 	|   |                                |   |
*	|___|________________________________|___|Y= 0
*   X=0                                     X= 3000
*
* Position Angulaire
* ------------------
*
*            Pi/2 rad
*               |
*               |
* Pi rad -------|------> 0 rad (position angulaire de départ)
*               |
*               |
*           -Pi/2 rad
*
*
*
********************************************************************/

//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waOdometrie, 64);

// Declare thread function for Odometrie (priority 11).
NIL_THREAD(Odometrie, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeOdometrie);  
    
    /*Environnement*/
	static float sommedDist = 0;


    /*Algo*/
		/*Sauvegarde codeurs*/
        pos.prec.encD  = pos.encD;
        pos.prec.encG = pos.encG;
        
		/*MAJ codeurs*/
        pos.encD = encD.read();
        pos.encG = encG.read(); //Inverser!! 

		/*Variation des mesures codeurs*/
	odo.delta.droit  = pos.encD - pos.prec.encD;
	odo.delta.gauche = pos.encG - pos.prec.encG;

		/*Variation des mesures*/
				//Angle
				odo.delta.angle = (((float)(odo.delta.droit-odo.delta.gauche))/((float)(LongueurEntreAxe)));
				//Distance
				odo.delta.dist = (float)(odo.delta.droit+odo.delta.gauche)/2;

		/*Calcul de l'angle et de la position*/
				//Nouvelle position en ticks
    
      				  //Ancien systeme serinus
      				  //pos.xtick += cosf(pos.arad/* + (odo.delta.angle/2)*/) * (odo.delta.dist);
      				  //pos.ytick += sinf(pos.arad/* + (odo.delta.angle/2)*/) * (odo.delta.dist);
    
                                pos.xtick += cosf(pos.arad) * (odo.delta.dist);
				pos.ytick += sinf(pos.arad) * (odo.delta.dist);

				//Conversion en mm
				 pos.x   = (signed int)(pos.xtick / (float)TickParmm);
				 pos.y   = (signed int)(pos.ytick / (float)TickParmm);

				//Angle en radian
				if(CouleurDepart == ROUGE)
					pos.arad += odo.delta.angle;
				else if(CouleurDepart == BLEU)
					pos.arad -= odo.delta.angle;

				//Modulo 2Pi (entre Pi et -Pi)
				if(pos.arad>constPI)   pos.arad-=2*constPI;
				if(pos.arad<=-constPI) pos.arad+=2*constPI;

                                //Conversion en dixieme de degree
                                pos.a = pos.arad * 3600 / (2*constPI);
                                
                /*Copmpteur d'itérations*/
                odo.iteration  ++;        
        
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print(":");
    #endif
    
    //Appel de la tache d'asservissement
    nilSemSignal(&wakeAsservissement);  
  }  
}


/* FONCTIONS ANNEXES */
void resetOdometrie(int x, int y, int a)
{
	nilSysLock();

        pos.x = x;
        pos.y = y;
        pos.a = a;
        pos.xtick = x*TickParmm;
        pos.ytick = y*TickParmm;
        pos.arad = 2*constPI*(float)a/3600;
        
        nilSysUnlock();
}

/**
	* @use : Acesseur en lecture de x (mm), necessaire car la variable n'est pas atomique
	*
	* @param : void
	*
	* @retun :
	*          - int : tmp = l'abscisse (x) de la position du robot en mm
	*
	*/
int getX()
{
/*Environnement*/
	float tmp;
/*Algo*/
	//GetResource(ResOdometrie);
	tmp = pos.x;
	//ReleaseResource(ResOdometrie);

	//Conversion en int et renvoi
	return (int)tmp;
}

/**
	* @use : Acesseur en lecture de y (mm), necessaire car la variable n'est pas atomique
	*
	* @param : void
	*
	* @retun :
	*          - int : tmp = l'ordonnee (y) de la position du robot en mm
	*
	*/
float getY()
{
/*Environnement*/
	float tmp;
	//GetResource(ResOdometrie);
	tmp = pos.y;
	//ReleaseResource(ResOdometrie);
	
	//Conversion en int et renvoi
	return (int)tmp;
}

/**
  * @use : retourne l'angle du robot en degres
  *
  * @param : void
  *
  * @retun : 
  * 	- int: tmp = l'angle du robot en degres
  *
  */
int getAngle()
{
/*Environnement*/
	float tmp;

/*Algo*/
	//GetResource(ResOdometrie);
	tmp = pos.arad;
	//ReleaseResource(ResOdometrie);

	//Conversion en centiemes de degres
	tmp *= 18000 / constPI;

	//Retour et conversion en int
	return (int)tmp;
}

char getTickParmm()
{
	return TickParmm;
}

unsigned int getLongueurEntreAxe()
{
	return LongueurEntreAxe;
}

/**
	* @use : Conversion tick -> mm pour x
	*
	* @param :
	*
	* @retun :
	*
	*/
void setXmm(unsigned int x)
{
	//GetResource(ResOdometrie);
	pos.xtick=x*TickParmm;
        pos.x = x;
	//ReleaseResource(ResOdometrie);

}

/**
	* @use : Conversion tick -> mm pour y
	*
	* @param :
	*
	* @retun :
	*
	*/
void setYmm(unsigned int y)
{
	//GetResource(ResOdometrie);
	pos.ytick=y*TickParmm;
        pos.y = y;
	//ReleaseResource(ResOdometrie);
}

/**
	* @use : Conversion degrée radian
	*
	* @param :
	*
	* @retun :
	*
	*/
void setAngle(float angledegree)
{
	//GetResource(ResOdometrie);
	pos.arad=(angledegree*constPI)/180;
        pos.a = angledegree*10;
	//ReleaseResource(ResOdometrie);
}

unsigned int getAxeEssieu()
{
	return AxeEssieu;
}

unsigned int getCentreEssieu()
{
	return CentreEssieu;
}
