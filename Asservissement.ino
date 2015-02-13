//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waAsservissement, 64);

// Declare thread function for Asservissement (priority 12).
NIL_THREAD(Asservissement, arg) 
{
  while (TRUE) 
  {
    //Attente du signal de réveil
    nilSemWait(&wakeAsservissement);  
    
    //Algo
        //Si on asservit le robot en distance
        asserv.dist.etat = ON;
        asserv.angle.etat = ON;        
        //geneTraj.dist.consigneFiltree = 0;
        //geneTraj.angle.consigneFiltree = 3.14;
        geneTraj.dist.phase = ATTENTE;
        geneTraj.angle.phase = ATTENTE;
        /*Schunt de la génération de trajectoire, suivi d'une consigne sonar*/        
            if(asserv.dist.etat == ON)
            {
                //Calcul distance parcourue
                //asserv.dist.variation = pos.encD;
                asserv.dist.variation = (float)((pos.encD - pos.prec.encD) + (pos.encG - pos.prec.encG))/2 ;   //Depuis la dernière fois
                asserv.dist.mesure += asserv.dist.variation;                                            //Depuis le début du mouvement

                //Calcul de la vitesse (ticks/s) du robot (temps d'echantillonage 5ms)
                asserv.dist.vitesse = (asserv.dist.variation)*1000/TpsEchCommande;

                //Calcul de l'ecart (ticks) avec la consigne
                asserv.dist.ecart = geneTraj.dist.consigneFiltree - asserv.dist.mesure;

                //Calcul de la commande
                asserv.dist.commande = PID_Distance(asserv.dist.ecart,asserv.dist.vitesse);
            }

        //Si on asservit le robot en orientation        
            if(asserv.angle.etat == ON)
            {
                //Calcul distance parcourue
                asserv.angle.variation = ((pos.encD - pos.prec.encD) - (pos.encG - pos.prec.encG))/((float)LongueurEntreAxe) ;  //Depuis la dernière fois
                if(CouleurDepart == ROUGE)                                                           //Depuis le début du mouvement
                  asserv.angle.mesure += atanf(asserv.angle.variation);
                else if(CouleurDepart == BLEU)
                  asserv.angle.mesure += atanf(asserv.angle.variation);

                //Calcul de la vitesse (rad/s) du robot (temps d'echantillonage 5ms)
                asserv.angle.vitesse = (asserv.angle.variation)*1000/TpsEchCommande;

                //Calcul de l'ecart (rad) avec la consigne
                asserv.angle.ecart = geneTraj.angle.consigneFiltree - asserv.angle.mesure;

                //Calcul de la commande
                asserv.angle.commande = PID_Angle(asserv.angle.ecart,asserv.angle.vitesse);
            }

    /*Regulation des commandes des 2 moteurs*/
        //Calcul commande en absolu
        if(CouleurDepart == ROUGE)
        {
            asserv.commande.abs.motG = fabs(asserv.dist.commande-asserv.angle.commande);
            asserv.commande.abs.motD = fabs(asserv.dist.commande+asserv.angle.commande);
        }
        else if(CouleurDepart == BLEU)
        {
            asserv.commande.abs.motG = fabs(asserv.dist.commande+asserv.angle.commande);
            asserv.commande.abs.motD = fabs(asserv.dist.commande-asserv.angle.commande);
        }

        //Zone de saturation (on conserve la proportion entre les deux commandes)
            if((asserv.commande.abs.motG > asserv.maxCommandeMoteur)||(asserv.commande.abs.motD > asserv.maxCommandeMoteur))
            {
                if(asserv.commande.abs.motG >= asserv.commande.abs.motD)
                    asserv.commande.correcteur = (float)asserv.maxCommandeMoteur/asserv.commande.abs.motG;
                else
                    asserv.commande.correcteur = (float)asserv.maxCommandeMoteur/asserv.commande.abs.motD;
            }
            else
              asserv.commande.correcteur = 1.0;
            //Calcul des commandes avec regulation
            if(CouleurDepart == ROUGE)
            {
                asserv.commande.motG = (signed int)((asserv.dist.commande-asserv.angle.commande)*asserv.commande.correcteur);
                asserv.commande.motD = (signed int)((asserv.dist.commande+asserv.angle.commande)*asserv.commande.correcteur);                
            }
            else if(CouleurDepart == BLEU)
            {
                asserv.commande.motG = (signed int)((asserv.dist.commande+asserv.angle.commande)*asserv.commande.correcteur);
                asserv.commande.motD = (signed int)((asserv.dist.commande-asserv.angle.commande)*asserv.commande.correcteur);                 
            }

	/*Envoi commande aux moteurs*/
        if(asserv.dist.etat == ON || asserv.angle.etat == ON){ //Si les deux asserv sont desactives, on envoit pas de commande aux moteurs
            if(MATCH_EN_COURS){
              mG.mot(asserv.commande.motG*coeffObstacle);
              mD.mot(asserv.commande.motD*coeffObstacle);
            }
            else
            {
              mG.mot(0);
              mD.mot(0);
            }
              
            
              // TEST COMMANDE
            /*if(MATCH_EN_COURS)
            {  coeffObstacle = 1;
                if(bouclePosition.sensDeplacement == AVANCE)
                {
            asserv.commande.motG = 1000.0*coeffObstacle;
            asserv.commande.motD = -1000.0*coeffObstacle;
              mG.mot(asserv.commande.motD);
              mD.mot(asserv.commande.motG);     
                }
                else
                {
            asserv.commande.motG = -1000.0*coeffObstacle;
            asserv.commande.motD = 1000.0*coeffObstacle;
              mG.mot(asserv.commande.motD);
              mD.mot(asserv.commande.motG);     ¹
              
                }
            }
            else
            {
            asserv.commande.motD = 0;
            asserv.commande.motG = 0;
              mG.mot(asserv.commande.motD);
              mD.mot(asserv.commande.motG);     
            }     */       
       }
        
    asserv.iteration++;
    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print(": ");
    #endif
  }  
}


/* FONCTIONS ANNEXES */
/********************************************************************************************************************************************************************/
/********************************************************************************************************************************************************************/

/****************************************************************************************************************/
/* FONCTION : PID_Distance(float ecart, float vitesse)
 * Entrée : ecart:  	float
 * Entrée : vitesse : float -JE NE SAIS PAS A QUOI CA SERT
 * Sortie : commande: 	float	-valeur de la commande pour un moteur
 * Utilitée: Modelise un regulateur PID pour l'asservissement en distance
 */
float PID_Distance(float ecart, float vitesse){
/*Environnement*/
    //La commande qui sortira du PID
        float commande=0.0;
	//Variaton de l'ecart (pour calculer la derivee)
        float variationEcart;
	//Sauvegarde ecart precedent
        static float previousEcart=0.0;



	//Coeffs PID en deplacement
        //Coefficients respectivement Proportionnel, Integral et Derivé, à ajjuster dans le header
        float kP=KP_D_Dep;
        float kI=KI_D_Dep;
        float kD=KD_D_Dep;

    //Coefs PID en regulation (Attente)
        if((geneTraj.dist.phase==0)||(geneTraj.dist.phase==4)){
            kP=KP_D_Att;
            kI=KI_D_Att;
            kD=KD_D_Att;
        }



//	if(fabs(ecart-variationEcart)>0.5){

/*Algo*/
    //Calcul de la variation de l'ecart (pour derivee)
        variationEcart= ecart-previousEcart;
	//Memorisation ecart
        previousEcart=ecart;
	//Calcul de la commande
        //Proportionel + Derivee
        commande = kP * ecart + kD * variationEcart;
        //Ajout conditionel de l'action integrale
//J'aime pas ça je commente (:
//        if ((commande<_commandeMax/2)&&(commande>-_commandeMax))
	commande += kI * asserv.dist.sommeEcart;

    //Calcul de la somme des ecarts depuis le debut de l'asser (necessaire pour l'integration)
        asserv.dist.sommeEcart = asserv.dist.sommeEcart + ecart;

//La saturation est faite dans la tache commande
//	if(commande>10000)	commande=10000;
//	if(commande<-10000)	commande=-10000;

//Saturation pour la phase de réupération (j'aime pas ça je commente)
//	if(geneTraj.dist.phase==4){
//		if(commande>1500)	commande=1500;
//		if(commande<-1500)	commande=-1500;
//	}

//	}

	return commande;
}

/****************************************************************************************************************/
/* FONCTION : PID_Angle(float ecart, float vitesse)
 * Entrée : ecart:  	float
 * Entrée : vitesse : float -JE NE SAIS PAS A QUOI CA SERT
 * Sortie : commande: 	float	-valeur de la commande pour un moteur
 * Utilitée: Modelise un regulateur PID pour l'asservissement en angle radian
 */
float PID_Angle(float ecart, float vitesse){
/*Environnement*/
    //La commande qui sortira du PID
        float commande=0.0;
	//Variaton de l'ecart (pour calculer la derivee)
        float variationEcart;
	//Sauvegarde ecart precedent
        static float previousEcart=0.0;

	//Coeffs PID en deplacement
        //Coefficients respectivement Proportionnel, Integral et Derivé, à ajjuster dans le header
        float kP=KP_O_Dep;
        float kI=KI_O_Dep;
        float kD=KD_O_Dep;

    //Coeffs PID en regulation (Attente)
        if((geneTraj.angle.phase==0)||(geneTraj.angle.phase==4)){
            kP=KP_O_Att;
            kI=KI_O_Att;
            kD=KD_O_Att;
        }

//	if(fabs(ecart-variationEcart)>0.05){

/*Algo*/
    //Calcul de la variation de l'ecart (pour derivee)
        variationEcart= ecart-previousEcart;
	//Memorisation ecart
        previousEcart=ecart;
	//Calcul de la commande
        //Proportionel + Derivee
        commande = kP * ecart + kD * variationEcart;
        //Ajout conditionel de l'action integrale
//J'aime pas ça je commente (:
//        if ((ecart<0.025)&&(ecart>-0.025)){
            commande += kI * asserv.angle.sommeEcart;
    //Calcul de la somme des ecarts depuis le debut de l'asser (necessaire pour l'integration)
		 asserv.angle.sommeEcart = asserv.angle.sommeEcart + ecart;

//Saturation pour la phase de réupération (encore une fois j'aime pas ça et je commente (: )
//	if(geneTraj.angle.phase==4){
//		if(commande>2000)	commande=2000;
//		if(commande<-2000)	commande=-2000;
//	}

//	}

	return commande;
}



/****************************************************************************************************************/
/* FONCTION : init_PID(void)
 * Entrée : void
 * Sortie : void
 * Utilitée: permet de réinitialiser les PID (somme des ecarts)
 */
void init_PID(void){
	//GetResource(ResCommande);
    //RAZ somme des ecarts
    	asserv.dist.sommeEcart=0.0; 	//RAZ somme des ecarts en distance (necessaire pour l'Integrateur du PID)
    	asserv.angle.sommeEcart=0.0; 		//RAZ somme des ecarts en angle (necessaire pour l'Integrateur du PID)
	//ReleaseResource(ResCommande);
}









/****************************************************************************************************************/
/* FONCTION : setAsservissementDistance( int asservissementDistance)
 * Entrée : char asservissementDistance	-une valeur égale à '0' pour ne pas asservir en distance
					-une valeur différente de '0' pour asservir en distance
 * Sortie : void
 * Utilitée: Permet d'asservir le robot en distance
 */
void setAsservissementDistance( char asservissementDistance){
	//GetResource(ResCommande);
	asserv.dist.etat= asservissementDistance; //si on asservie le robot en distance ou pas ('0' pour ne pas asservir)
	//ReleaseResource(ResCommande);
}

/****************************************************************************************************************/
/* FONCTION : setAsservissementOrientation( int asservissementOrientation)
 * Entrée : char asservissementOrientation	-une valeur égale à '0' pour ne pas asservir n orientation
						-une valeur différente de '0' pour asservir en orientation
 * Sortie : void
 * Utilitée: Permet d'asservir le robot en orientation
 */
void setAsservissementOrientation( char asservissementOrientation){
	//GetResource(ResCommande);
	asserv.angle.etat= asservissementOrientation; //si on asservie le robot en orientation ou pas ('0' pour ne pas asservir)
	//ReleaseResource(ResCommande);
}



/****************************************************************************************************************/
/* FONCTION : int getEcartDistance()
 * Entrée : void
 * Sortie : int tmp	-valeur entiere de l'ecart en distance
 * Utilitée: Permet d'obtenir la valeur de l'ecart en distance en entier
 */
int getEcartDistance(){
	int tmp;
	//GetResource(ResCommande);
	tmp=(int)asserv.dist.ecart;
	//ReleaseResource(ResCommande);
	return (int) tmp; //précision au mm pres
}

/****************************************************************************************************************/
/* FONCTION : int getEcartAngle()
 * Entrée : void
 * Sortie : int tmp valeur entiere de l'ecart en angle
 * Utilitée: Permet d'obtenir la valeur de l'ecart en angle en entier
 */
int getEcartAngle(){
	int tmp;
	//GetResource(ResCommande);
	tmp= (int)asserv.angle.ecart;
	//ReleaseResource(ResCommande);
	return (int) tmp; //précision au degre pres
}
