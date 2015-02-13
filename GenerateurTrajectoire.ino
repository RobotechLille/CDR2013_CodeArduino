//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waGenerateurTrajectoire, 64);

// Declare thread function for GenerateurTrajectoire (priority 21).
NIL_THREAD(GenerateurTrajectoire, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeGenerateurTrajectoire);  
    
/*Environnement*/
        //Consigne de vitesse
	static float consigneVitesseDep = 0;
	static float consigneVitesseAngle = 0;
		//Consignes d'acceleration	
	static float accelerationDep = 0;
	static float accelerationAngle = 0;
		//Distances restants à parcourir (relatives et absolues)
	float distanceDepRestante = 0;
	float distanceDepRestanteAbs = 0;
	float distanceAngleRestante = 0;
	float distanceAngleRestanteAbs = 0;
		//Vitesse et vitesse angulaire du robot (absolu)
	signed int vitesseAngleAbsolue = 0;
	signed int vitesseDepAbsolue = 0;
	//Seuils de fin de phase de deplacement
	// float seuilConsigneDepDebutDeceleration = (consigneVitesseDep*geneTraj.dist.max.tpsAcceleration*(asserv.dist.vitesse)) / (1*geneTraj.dist.max.vitesse*TickParmm()*1000); // 200 | 20cm
	float tpsDeceleration = ((consigneVitesseDep * geneTraj.dist.max.tpsAcceleration)+TpsEchGeneTraj) / geneTraj.dist.max.vitesse;
	float seuilConsigneDepDebutDeceleration = (tpsDeceleration*(asserv.dist.vitesse)) / (TickParmm*1500); // 200 | 20cm

	float seuilConsigneDepFinDeceleration = 2*TpsEchGeneTraj*asserv.dist.vitesse / (TickParmm*1000); //1cm

	// const unsigned int seuilConsigneDepFinDeceleration = 0; //1cm
	const unsigned int seuilConsigneDepPrecision=40;
		//Angle en centiemes de degre
	const unsigned int seuilConsigneAngleDebutDeceleration=1000; //0.4
	const unsigned int seuilConsigneAngleFinDeceleration=300; //0.015 rad=0.86degré
	const unsigned int seuilConsigneAnglePrecision=500;
		//Timers et nombres d'iteration
	static signed int iDep=0; //nombre d'itinération pour la phase 4 du déplacement
	static signed int iAngle=0;
	static signed int timerFinAngle=0;
	static signed int timerFinDep=0;
	static signed int timerAngle_Phase_Deceleration=0;
	static signed int timerDep_Phase_Deceleration=0;
	static unsigned int it;



if(geneTraj.dist.phase == CONSTANTE && 1)
{
	saveSeuil = seuilConsigneDepDebutDeceleration;
	saveRap = (consigneVitesseDep*geneTraj.dist.max.tpsAcceleration) / (geneTraj.dist.max.vitesse);
	saveVit = asserv.dist.vitesse;
	saveCV = asserv.dist.mesure;
}


/*Algo*/

	/*Cas d'une demande d'arret de deplacement*/
	if(_error == STOP)
		arretDeplacement();

	/*Asserv de distance*/
	if(asserv.dist.etat && 0)
	{
		//Mesure de la distance restance a parcourir en mm
		distanceDepRestante = geneTraj.dist.consigne - asserv.dist.mesure / TickParmm ;
		//En absolu
		distanceDepRestanteAbs = fabs(geneTraj.dist.consigne - asserv.dist.mesure/TickParmm);
		//Idem pour la vitesse en mm/s
		vitesseDepAbsolue = fabs(asserv.dist.vitesse);

		switch(geneTraj.dist.phase)
		{
			/*Phase d'attente (on est en regulation sur la position actuelle)*/
			case ATTENTE:			
				//Attente d'une nouvelle consigne pour passer en phase suivante
				if(geneTraj.dist.consigne != geneTraj.dist.consigneFiltree  && _error != ARRET) 
					geneTraj.dist.phase = ACCELERATION;
				//Sinon on reste sur place
				else 
					//La consigne filtree est en ticks (conversion)
					geneTraj.dist.consigneFiltree = (long)(geneTraj.dist.consigne) * TickParmm;

				//Initialisation des timers
					timerFinDep=0;
					timerDep_Phase_Deceleration=0;

				//RAZ des consignes de vitesse
					consigneVitesseDep = 0;

				//RAZ de la commande max
					asserv.maxCommandeMoteur = 10000;
			break;


			/*Phase d'acceleration*/
			case ACCELERATION:
				//On fixe l'acceleration
				accelerationDep = (float)(geneTraj.dist.max.vitesse * TpsEchGeneTraj) / geneTraj.dist.max.tpsAcceleration; 
				//La consigne doit s'incrementer
				consigneVitesseDep += accelerationDep;

				/*Generation de la consigne de distance filtree*/
					//Consigne positive
					if( geneTraj.dist.consigne>0)
						//La consigne filtre compense le deplacement
						geneTraj.dist.consigneFiltree = asserv.dist.mesure + consigneVitesseDep;
					//Consigne negative
					else		   
						//La consigne filtre compense le deplacement
						geneTraj.dist.consigneFiltree = asserv.dist.mesure - consigneVitesseDep;

				/*Passage aux phases suivantes*/
					//Soit on depasse la vitesse max autorisee
					if( consigneVitesseDep > geneTraj.dist.max.vitesse)
					{
						//Saturation de la consigne de vitesse
						consigneVitesseDep  = geneTraj.dist.max.vitesse;
						//Passage en vitesse constante
						geneTraj.dist.phase = CONSTANTE;
					}
					//Soit on atteint deja le seuil de debut de deceleration
					if(		(distanceDepRestante < seuilConsigneDepDebutDeceleration && geneTraj.dist.consigne > 0)
						||	(distanceDepRestante > seuilConsigneDepDebutDeceleration && geneTraj.dist.consigne < 0))
					{
						//RAZ du timer de la phase de deceleration
						timerDep_Phase_Deceleration = 0;
						//Passage en phase de deceleration
						geneTraj.dist.phase = DECELERATION;
					} 
			break;


			/*Phase a vitesse constante*/
			case CONSTANTE:
				/*Generation de la consigne de distance filtree*/
					//MAJ de la Consigne de vitesse
					consigneVitesseDep = geneTraj.dist.max.vitesse;
					//Consigne positive
					if( geneTraj.dist.consigne > 0)
						//La consigne filtre compense le deplacement
						geneTraj.dist.consigneFiltree = asserv.dist.mesure + consigneVitesseDep;
					//Consigne negative
					else 		   
						//La consigne filtre compense le deplacement
						 geneTraj.dist.consigneFiltree = asserv.dist.mesure - consigneVitesseDep;

				/*Passage a la phase suivante*/
					//On atteint deja le seuil de debut de deceleration
					if(		(distanceDepRestante < seuilConsigneDepDebutDeceleration && geneTraj.dist.consigne > 0)
						||	(distanceDepRestante > seuilConsigneDepDebutDeceleration && geneTraj.dist.consigne < 0))
					{
						//RAZ du timer de la phase de deceleration
						timerDep_Phase_Deceleration = 0;
						//Passage en phase de deceleration
						geneTraj.dist.phase = DECELERATION;
					}
			break;


			/*Phase de deceleration*/
			case DECELERATION:
				//On decelere, l'acceleration est negative (on decelere 2x plus vite qu'on accelere)
				accelerationDep= - (float)(geneTraj.dist.max.vitesse * TpsEchGeneTraj) / geneTraj.dist.max.tpsAcceleration;  
				//La consigne doit decrementer
				consigneVitesseDep += accelerationDep;

				/*Generation de la consigne de distance filtree*/
					//Consigne positive
					if( geneTraj.dist.consigne > 0)
						//La consigne filtre compense le deplacement
						geneTraj.dist.consigneFiltree = asserv.dist.mesure + consigneVitesseDep;
					//Consigne negative
					else  		   
						//La consigne filtre compense le deplacement
						 geneTraj.dist.consigneFiltree = asserv.dist.mesure - consigneVitesseDep;

				/*Passage en recuperation*/
					//Si la consigne de vitesse devient negative (les roues ne doivent pas tourner en sens inverse)
					//Si on depasse le seuil de fin de deceleration
					//Si cela fait trop longtemps que l'on se trouve en phase de deceleration (200*20 = 4000ms = 4s)
				if(		(consigneVitesseDep < 0)
					||	((distanceDepRestante < seuilConsigneDepFinDeceleration) && geneTraj.dist.consigne > 0)
					||	((distanceDepRestante > seuilConsigneDepFinDeceleration) && geneTraj.dist.consigne < 0)
					||	(timerDep_Phase_Deceleration > 200))
				{
					//On fixe la consigne a la consigne finale
					geneTraj.dist.consigneFiltree = (long)geneTraj.dist.consigne;

					//Passage en phase de recuperation
					geneTraj.dist.phase = RECUPERATION;

					//RAZ du calcul necessaire a l'integrale du PID
						//(cette somme, a cause des phases d'acceleration, atteint des extremes
						// il faut la RAZ pour eviter d'enormes depassement en phase de recuperation)
					asserv.dist.sommeEcart = 0.0;

					//Reset du timer
					timerDep_Phase_Deceleration=0;
				}
				else
					/*Incrementation timer*/
					timerDep_Phase_Deceleration++;
			break;


			/*Phase de recuperation*/
			case RECUPERATION:
			//Saturation plus importante
				asserv.maxCommandeMoteur = 4000;

			//On fixe la consigne a la consigne finale
				geneTraj.dist.consigneFiltree = (long)geneTraj.dist.consigne;

			//On compte le nombre de fois consecutives ou l'on reste sous le seuil de precision
				if(distanceDepRestanteAbs < seuilConsigneDepPrecision)
					iDep++;
				else	
					iDep=0;


			//Si on est reste sous le seuil (10*20 = 0.2s) ou que le timer touche a sa fin (200*20 = 4s)
				if((iDep >= 10) || (timerFinDep > 200))
				{
					//Retour en phase d'attente
					geneTraj.dist.phase = ATTENTE;

					//Les consignes sont nulles et l'on fixe le nouveau point de depart ici(on entre donc en regulation)
					geneTraj.dist.consigne=0;
					geneTraj.dist.consigneFiltree=0;
					fixation_position_depart();

					//Flag d'arrive a la consigne
					geneTraj.dist.fin = ARRIVE;					

					//Reset du timer et de l'iteration
					timerFinDep=0;
					iDep=0;
				}
				else
					/*Incrementation timer*/
					timerFinDep++;


				//Conversion en ticks
				geneTraj.dist.consigneFiltree *=  (float)TickParmm;

			break;
		}
	}


	/*Asserv en orientation*/
	if(asserv.angle.etat && 0)
	{
		//Calcul de la distance restante (via la distance parcourue calculee par la tache Commande)
		distanceAngleRestante = geneTraj.angle.consigne - asserv.angle.mesure;
		//Passage en absolu
		distanceAngleRestanteAbs = fabs(distanceAngleRestante);
		//Idem pour la vitesse angulaire (en degre/s)
		vitesseAngleAbsolue = fabs(asserv.angle.vitesse);

		/*Algo d'etat*/
		switch(geneTraj.angle.phase)
		{

			/*Phase d'attente (on est en regulation sur la position angulaire actuelle)*/
			case ATTENTE:
				//Attente d'une nouvelle consigne pour passer en phase suivante
					//A condition de ne pas etre sous l'effet de la boucle de position
				if((geneTraj.angle.consigne != geneTraj.angle.consigneFiltree) && (bouclePosition.etat == OFF)  && _error != ARRET)
					geneTraj.angle.phase = ACCELERATION;
				//Sinon on reste sur place
				else 
					geneTraj.angle.consigneFiltree = geneTraj.angle.consigne;

				//Initialisation des timers
					timerFinAngle = 0;
					timerAngle_Phase_Deceleration = 0;

				//RAZ des consignes de vitesse
					consigneVitesseAngle = 0;

			break;


			/*Phase d'acceleration*/
			case ACCELERATION:
				//On fixe l'acceleration au maximum
				accelerationAngle = (float)(geneTraj.angle.max.vitesse * TpsEchGeneTraj) / geneTraj.angle.max.tpsAcceleration; 
				//La consigne doit s'incrementer
				consigneVitesseAngle += accelerationAngle;

				/*Generation de la consigne d'angle filtree*/
					//Consigne positive
					if( geneTraj.angle.consigne > 0)
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure + consigneVitesseAngle;
					//Consigne negative
					else 		   
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure - consigneVitesseAngle;

				/*Passage aux phases suivantes*/
					//Soit on depasse la vitesse max autorisee
					if( consigneVitesseAngle > geneTraj.angle.max.vitesse)
					{ 
						//Saturation de la consigne de vitesse
						consigneVitesseAngle = geneTraj.angle.max.vitesse;
						//Passage en vitesse constante
						geneTraj.angle.phase = CONSTANTE;
					}
					//Soit on atteint deja le seuil de debut de deceleration
					else if(distanceAngleRestanteAbs < seuilConsigneAngleDebutDeceleration)
					{
						//RAZ du timer de la phase de deceleration
						timerAngle_Phase_Deceleration = 0;
						//Passage en phase de deceleration
						geneTraj.angle.phase = DECELERATION;
					}
			break;


			/*Phase a vitesse constante*/
			case CONSTANTE:
				/*Generation de la consigne d'angle filtree*/
					//Consigne positive
					if( geneTraj.angle.consigne > 0)
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure + consigneVitesseAngle;
					//Consigne negative
					else 		   
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure - consigneVitesseAngle;

				/*Passage a la phase suivante*/
					//On atteint deja le seuil de debut de deceleration
					if(distanceAngleRestanteAbs < seuilConsigneAngleDebutDeceleration)
					{
						//RAZ du timer de la phase de deceleration
						timerAngle_Phase_Deceleration = 0;
						//Passage en phase de deceleration
						geneTraj.angle.phase = DECELERATION;
					}
			break;


			/*Phase de deceleration*/
			case DECELERATION:
				//On decelere
				accelerationAngle = - (float)(geneTraj.angle.max.vitesse * TpsEchGeneTraj) / geneTraj.angle.max.tpsDeceleration; 
				//La consigne doit decrementer
				consigneVitesseAngle += accelerationAngle;

				/*Generation de la consigne d'angle filtree*/
					//Consigne positive
					if( geneTraj.angle.consigne > 0)
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure + consigneVitesseAngle;
					//Consigne negative
					else 		   
						//La consigne filtre compense le deplacement
						geneTraj.angle.consigneFiltree = asserv.angle.mesure - consigneVitesseAngle;

				/*Passage en recuperation*/
					//Si la consigne de vitesse devient negative (les roues ne doivent pas tourner en sens inverse)
					//Si on depasse le seuil de fin de deceleration
					//Si cela fait trop longtemps que l'on se trouve en phase de deceleration (200*20 = 4000ms = 4s)
				if( 	(consigneVitesseAngle < 0)
					||  (distanceAngleRestanteAbs < seuilConsigneAngleFinDeceleration)
					||  (timerAngle_Phase_Deceleration >200))
				{
					//On fixe la consigne a la consigne finale
					geneTraj.angle.consigneFiltree = geneTraj.angle.consigne;

					//Passage en phase de recuperation
					geneTraj.angle.phase = RECUPERATION;

					//RAZ du calcul necessaire a l'integrale du PID
						//(cette somme, a cause des phases d'acceleration, atteint des extremes
						// il faut la RAZ pour eviter d'enormes depassement en phase de recuperation)
					asserv.angle.sommeEcart = 0.0;

					//Reset du timer
					timerAngle_Phase_Deceleration = 0;
				}
				else
					/*Incrementation timer*/
					timerAngle_Phase_Deceleration++;

			break;


			/*Phase de recuperation*/
			case RECUPERATION:
			//On compte le nombre de fois où l'on reste sous le seuil de precision
				if(distanceAngleRestanteAbs < seuilConsigneAnglePrecision)	
					iAngle++;
				else 
					iAngle=0;
			//Si on est reste sous le seuil (10*20 = 0.2s) ou que le timer touche a sa fin (200*20 = 4s)
				if((iAngle >= 10)||(timerFinAngle>200))
				{
					//Retour en phase d'attente
					geneTraj.angle.phase = ATTENTE;

					//Les consignes sont nulles et l'on fixe le nouveau point de depart ici(on entre donc en regulation)
					geneTraj.angle.consigne=0;
					geneTraj.angle.consigneFiltree=0;
					fixation_position_depart();

					//Flag d'arrive a la consigne
					geneTraj.angle.fin = ARRIVE;

					//Reset du timer et de l'iteration
					timerFinAngle=0;
					iAngle=0;
				}
				else
					/*Incrementation timer*/
					timerFinAngle++;
			break;
		}
	}
	
	it++;
    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print("| ");
    #endif
  }  
}



/* FONCIONS ANNEXES */

//////////////////////////////////////////////////////////////////////////////////////////////////
	//Fonctions de deplacement de bas niveau//--------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************************************/
/****************************************************************************************************************/



/**
 * @use : avance rectiligne avec freinage d'une distance donnee (en mm)
 *
 * @param :
 *     	- float: distance = distance a parcourir en mm
 *		- float: vitesse  = vitesse maximale du deplacement en % (0 à 100)
 *
 * @retun : 
 *	 	- char: error     = retour d'erreur (0 si tout s'est bien passe)
 *
 */
void avance_Avec_Freinage(float distance,float vitesse, char PositionLoop, char*error)
{
/*Erreur nulle en debut de fontion*/
	*error=0;

	if(distance!=0)
	{
	/*Desactivation des asservissements*/
		setAsservissementDistance(0);
		setAsservissementOrientation(0);
	/*Reinitialisations*/
		fixation_position_depart(); 
		init_PID();
	/*Configuration des parametres du deplacement*/
		setVitesseDepMax(vitesse);
		setConsigneDep(distance);
		setConsigneAngle(0);
	/*Reactivation des asservissements*/
		setAsservissementDistance(1);
		setAsservissementOrientation(1);
	/*Lancement de la boucle de position*/
		setSensDeplacement(AVANCE);
		setEtatBouclePosition(PositionLoop);
	/*Attente de fin de deplacement*/
		geneTraj.dist.fin = EN_COURS;
		//while((geneTraj.dist.fin != ARRIVE)&&(*error==0));
	/*Arret de la boucle de position*/
		setEtatBouclePosition(OFF);
	}
}



/**
 * @use : recul rectiligne avec freinage d'une distance donnee (en mm)
 *
 * @param :
 *     	- float: distance = distance a parcourir en mm
 *		- float: vitesse  = vitesse maximale du deplacement en % (0 à 100)
 *
 * @retun : 
 *	 	- char: error     = retour d'erreur (0 si tout s'est bien passe)
 *
 */
void recule_Avec_Freinage(float distance, float vitesse, char PositionLoop, char*error)
{
/*Erreur nulle en debut de fontion*/
	*error=0;

	if(distance!=0)
	{
	/*Inversion*/
		distance = -distance;
	/*Desactivation des asservissements*/
		setAsservissementDistance(0);
		setAsservissementOrientation(0);
	/*Reinitialisations*/
		fixation_position_depart(); 
		init_PID();
	/*Configuration des parametres du deplacement*/
		setVitesseDepMax(vitesse);
		setConsigneDep(distance);
		setConsigneAngle(0);
	/*Reactivation des asservissements*/
		setAsservissementDistance(1);
		setAsservissementOrientation(1);
	/*Lancement de la boucle de position*/
		setSensDeplacement(RECULE);
		setEtatBouclePosition(PositionLoop);

	/*Attente de fin de deplacement*/
		geneTraj.dist.fin = EN_COURS;
		while((geneTraj.dist.fin != ARRIVE)&&(*error==0));
	/*Arret de la boucle de position*/
		setEtatBouclePosition(OFF);
	}
}

void recule_Jusque_Blocage(char*error){
/*Environnement*/
	unsigned int compteurBlocageG=0, compteurBlocageD=0;
	signed long cGauche=0, cDroit=0, cGauchePrecedent=0, cDroitPrecedent=0;
	unsigned int avanceGauche,avanceDroite=0;

	#define VITESSE_RECUL_CALAGE 6000
	#define SEUIL_PATINAGE 30
	#define SEUIL_BLOCAGE 100
	#define PAUSE_MS	5

/*Algo*/

	//Desactivation de l'asservissement
	setAsservissementDistance(0);
	setAsservissementOrientation(0);

	//Premiere lecture de la valeur des codeurs
                cGauche = encG.read();
                cDroit = encD.read();

	//On recule
        mD.mot(-VITESSE_RECUL_CALAGE);
        mG.mot(-VITESSE_RECUL_CALAGE-1000);

//Recul double
	do{
		//Pause systeme
		nilThdSleepMilliseconds(PAUSE_MS);

		//Sauvegarde des mesures precedentes
		cGauchePrecedent=cGauche;
		cDroitPrecedent=cDroit;

		//Lecture de la valeur des codeurs
                cGauche = encG.read();
                cDroit = encD.read();

		//On regarde de combien on a avance sur le codeur gauche
		avanceGauche=abs(cGauche-cGauchePrecedent);
		//On regarde de combien on a avance sur le codeur droit
		avanceDroite=abs(cDroit-cDroitPrecedent);

	/*Filtre pour la prise en compte d'un patinage*/
		//Si on l'avance sur la roue gauche est inferieur a SEUIL_PATINAGE, on incremente le compteur de blocage
		if(avanceGauche<SEUIL_PATINAGE) 
			compteurBlocageG++;
		else
			compteurBlocageG = 0;
		//Si on l'avance sur la roue gauche est inferieur a SEUIL_PATINAGE, on incremente le compteur de blocage
		if(avanceDroite<SEUIL_PATINAGE) 
			compteurBlocageD++;
		else
			compteurBlocageD = 0;

	/*Arrêt des roues en cass de patinage*/
		//Quand on est bloqué on stop la roue
		if(compteurBlocageD >= SEUIL_BLOCAGE){
				mD.mot(0);
			}
		else{
			//controlMoteurDroit(-VITESSE_RECUL_CALAGE);			
		}
		//Quand on est bloqué on stop la roue
		if(compteurBlocageG >= SEUIL_BLOCAGE){
			mG.mot(0);	
		}
		else{
			//controlMoteurGauche(-VITESSE_RECUL_CALAGE);
		}

		// //Si on l'avance sur les deux roues est inferieur a SEUIL_PATINAGE, on incremente le compteur de blocage
		// if((avanceGauche<SEUIL_PATINAGE)&&(avanceDroite<SEUIL_PATINAGE)) 
		// 	compteurBlocage++;
		// else
		// 	compteurBlocage = 0;
	}while(compteurBlocageD<SEUIL_BLOCAGE && compteurBlocageG<SEUIL_BLOCAGE); //On attend d'etre bloques 'SEUIL_BLOCAGE' fois

	//On est cales -> Arret des moteurs
        mD.mot(0);
        mG.mot(0);

	//Reactivation de l'asservissement
	setAsservissementDistance(1);
	setAsservissementOrientation(1);
}

/**
  * @use : rotation avec freinage d'un angle relatif à la position actuelle du robot
  *
  * @param :
  *     - float: angle     = angle a parcourir en centiemes de degre
  *		- int: vitesse     = vitesse maximale du deplacement en % (0 à 100)
  *
  * @retun : 
  *		- char: error      = retour d'erreur (1 si erreur, 0 si ok)
  *
  */
void rotation_De(float angle, int vitesse, char*error)
{
/*Erreur nulle en début de fonction*/
	*error=0;

	if(angle!=0)
	{
		/*Desactivation des asservissement*/
			setAsservissementDistance(0);
			setAsservissementOrientation(0);
		/*Reinitialisations*/
			fixation_position_depart();
			init_PID();
		/*Configuration des parametres du deplacement*/
			setVitesseAngleMax(vitesse);
			setConsigneDep(0);
			//Angle en centiemes de degres
			setConsigneAngle(angle);
		/*Reactivation des asservissements*/
			setAsservissementDistance(1);
			setAsservissementOrientation(1);
		/*Attente de fin de deplacement*/
			geneTraj.angle.fin = EN_COURS;
			while((geneTraj.angle.fin != ARRIVE)&&(*error==0));
	}
}



/**
  * @use : rotation avec freinage d'un angle relatif au repere de la table de jeu
  *
  * @param :
  *    	- float: angle_final = angle recherche en degres
  *		- int: vitesse       = vitesse maximale du deplacement en % (de 0 a 100)	
  *
  * @retun : 
  *	- char: error            = retour d'erreur (0 si tout s'est bien passe)
  *
  */
void orientation_De(float angle_final, int vitesse, char *error)
{
/*Environnement*/
	//Angle actuel du robot sur le repere de la table(obtenu via l'odometrie)
	int current_angle = getAngle();
	//Angle a parcourir depuis la position du robot
	float angle=0;

/*Algo*/
	/*Calcul de l'angle relatif au robot a parcourir*/
	angle = angle_final - current_angle;
	/*Conversion modulo 2Pi ]-Pi,Pi]*/
	if(angle>18000) 	angle -= 36000;
	else if(angle<=-18000)	angle += 36000;

	printf("\n\n\nANGLE FIN: %d | DeB : %d| Ang : %d\n\n\n",(int)angle_final,(int)current_angle,(int)angle);

	rotation_De(angle, vitesse, error);
}





//////////////////////////////////////////////////////////////////////////////////////////////////
	//Fonctions de recherche de consignes avec coordonnees//------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/



/**
  * @use : indique la distance en mm a parcourir pour atteindre le point (X,Y)
  *
  * @param :
  *     - int: coordX           = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY           = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *
  * @retun :
  *		- int: distanceConsigne = la distance a parcourir en mm
  *
  */
int rechercheDistanceConsigne(int coordX, int coordY)
{
/*Environnement*/
	//Distance a parcourir en mm
	int distanceConsigne=0;
	//Position actuelle du robot en mm (provient de l'odometrie)
	int posX = getX();
	int posY = getY();

/*Algo*/
	//Calcul de la distance selon le Th. de Pythagore
	distanceConsigne = sqrt( ((coordX-posX)*(coordX-posX)) + ((coordY-posY)*(coordY-posY)) );

	//Retour resultat
	return distanceConsigne;
}



/**
  * @use : indique l'angle en degres a parcourir pour s'orienter vers le point (X,Y)
  *
  * @param :
  *     - int: coordX        = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY        = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *
  * @retun : 
  *		-int: angleConsigne  = l'angle a parcourir en degres
  *
  * NB : On utilisera des changements de repère ainsi que la notion de repère polaire et les complexes pour trouver l'angle de consigne
  *
  */
int rechercheAngleConsigne(float coordX, float coordY)
{
/*Environnement*/
	//Position actuelle du robot en mm et en degres (provient de l'odometrie)
	int posX = getX();
	int posY = getY();
	int angleDep = getAngle();
	//Angle a parcourir en degres
	int angleConsigne = 0;
	//Angle a parcourir (en radians) si le robot a une orientation de 0degres
	float angle0 = 0;
	//Aides de calculs
		float valeurAjoute=0;
		float rapport=0;
		//On se place dans le plan complexe
		int Re = coordX - posX;
		int Im = coordY - posY;

/*Algo*/
	/*Calcul de l'angle a parcourir en radians pour une orientation initialle nulle*/
		//Gestion des cas particuliers (plus simples)
		if(Re==0||Im==0)
		{
			//Cas du point O
			if((Re==0)&&(Im==0))    angle0 = 0;
			//Reel pur
			else if(Re==0)
			{
				if(Im>0)            angle0 = constPI/2;
				else                angle0 = -constPI/2;
			}
			//Imaginaire pur
			else if(Im==0)
			{
			    	if(Re>0)        angle0 = 0;
			    	else            angle0 = constPI;
			}
		}
		//Autres cas (plus lourd a calculer)
		else
		{
			if(Re<0)
			{
			    if(Im>0)
					valeurAjoute = constPI;
				else
					valeurAjoute = -constPI;
			}
			//Calcul de l'angle
			rapport = (float)Im/Re;
			angle0 = atanf(rapport);
			angle0 += valeurAjoute;
		}

	/*Calcul de l'angle a parcourir en degres*/
		//Conversion en centiemes de degres
		angle0 *= 18000 / constPI;

		//Calcul
		angleConsigne = (int)angle0 - angleDep;

		//Modulo 360degres ]-180,180]
		if(angleConsigne> 18000)           angleConsigne -= 36000;
		else if(angleConsigne<=-18000)     angleConsigne += 36000;

	/*Retour de l'angle a parcourir en degres*/
	return angleConsigne;
}





//////////////////////////////////////////////////////////////////////////////////////////////////
	//Fonctions de deplacement avec coordonnees//--------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/



/**
  * @use : rotation avec freinage vers le point (X,Y)
  *
  * @param :
  *     - int: coordX        = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY        = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *		- int: vitesse       = vitesse maximale du deplacement en % (de 0 a 100)
  *
  * @retun : 
  *		- char: error        = retour d'erreur (0 si tout s'est bien deroule)
  *
  */
void rotation_Vers_Le_Pt(int coordX,int coordY,int vitesse, char*error)
{
/*Environnement*/
	//Angle a parcourir en degres
	int consigneAngle = 0;

/*Algo*/
	//Calcul de l'angle a parcourir en radian
	consigneAngle = rechercheAngleConsigne(coordX, coordY);
	//Execution du deplacement
	rotation_De(consigneAngle,vitesse,error);
}



/****************************************************************************************************************/
/* FONCTION : rotation_Arriere_Vers_Le_Pt(float coordX,float coordY,float vitesse, char*error)
 * Entrée : 	float	coordX		-coordonnée en x du point d'arrivée du robot
		float 	coordY		-coordonnée en y du point d'arrivée du robot
	    	float 	vitesse		vitesseMax du robot (valeur comprise entre [0;100])
 * Sortie : 	char error
 * Utilitée: le robot tourne le dos au pt de coordonnée X Y
 */
/**
  * @use : pour le moteur2
  *
  * @param :
  *     - int: coordX        = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY        = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *		- int: vitesse       = vitesse maximale du deplacement en % (de 0 a 100)
  *
  * @retun : 
  *		- char: error        = le retour d'erreur (0 lorsque tout est ok)
  *
  */
void rotation_Arriere_Vers_Le_Pt(int coordX, int coordY,int vitesse, char*error)
{
	int consigneAngle=0;	//valeur de la consigne d'orientation en radian a trouver
	char erreur=0;

	consigneAngle=rechercheAngleConsigne(coordX,coordY);//obtenir la consigne d'orientation en radian
	consigneAngle=consigneAngle+18000;
	if(consigneAngle>18000) consigneAngle=consigneAngle-36000;
	rotation_De(consigneAngle,vitesse,&erreur);
	*error=erreur;
}



/****************************************************************************************************************/
/* FONCTION : avance_Rectiligne_Avec_Freinage_Vers_Le_Pt(float coordX, float coordY, float vitesse, char*error)
 * Entrée : 	float	coordX		-coordonnée en x du point d'arrivée du robot
		float 	coordY		-coordonnée en y du point d'arrivée du robot
	    	float 	vitesse		vitesseMax du robot (valeur comprise entre [0;100])
 * Sortie : 	char 	error		indicateur permettant de savoir si le déplacement s'est corretement déroulé ("0":aucun problème, "1": il y a eu un un problème)
 * Utilitée: déplacement rectiligne du robot vers le pt désiré avec freinage
 */
/**
  * @use : deplacement rectiligne avec freinage vers le point (X,Y)
  *
  * @param :
  *     - int: coordX        = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY        = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *		- int: vitesse       = vitesse maximale du deplacement en % (de 0 a 100)
  *
  * @retun : 
  *		- char: error        = le retour d'erreur (0 lorsque tout est ok)
  *
  */
void avance_Rectiligne_Avec_Freinage_Vers_Le_Pt(int coordX, int coordY, int vitesse, char*error)
{
	int consigneDistance=0;	//valeur de la consigne en distance en mm a trouver

	rotation_Vers_Le_Pt(coordX,coordY, vitesse, error); 		//rotation vers le pt de coordonnée désiré avec freinage
	consigneDistance=rechercheDistanceConsigne(coordX,coordY); 	//obtenir la consigne de deplacement
	//Configuration des points pour la boucle de position
	setXvoulu(coordX);
	setYvoulu(coordY);
	avance_Avec_Freinage(consigneDistance, vitesse, OFF, error);// avance tout droit jusqu'au pt de coordonnée désiré avec freinage
}


/**
  * @use : pour le moteur2
  *
  * @param :
  *     - int: coordX        = l'abscisse de la coordonnee a atteindre (axe des X)
  *     - int: coordY        = l'ordonnee de la coordonnee a atteindre (axe des Y)
  *		- int: vitesse       = vitesse maximale du deplacement en % (de 0 a 100)
  *
  * @retun : 
  *		- char: error        = le retour d'erreur (0 lorsque tout est ok)
  *
  */
void avance_Rectiligne_En_Marche_Arriere_Avec_Freinage_Vers_Le_Pt(int coordX, int coordY, int vitesse, char*error)
{
	float consigneDistance=0;	//valeur de la consigne en distance en mm a trouver

	rotation_Arriere_Vers_Le_Pt(coordX,coordY, vitesse, error); 		//rotation vers le pt de coordonnée désiré avec freinage
	consigneDistance=rechercheDistanceConsigne(coordX,coordY); 	//obtenir la consigne de deplacement
	//Configuration des points pour la boucle de position
	setXvoulu(coordX);
	setYvoulu(coordY);
	recule_Avec_Freinage(consigneDistance, vitesse,OFF, error);// avance tout droit jusqu'au pt de coordonnée désiré avec freinage
}





//////////////////////////////////////////////////////////////////////////////////////////////////
	//Fonctions de calage et d'initialisation d'un point//--------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/


void arretDeplacement()
{
	//Ordre bien compris
	_error = ARRET;

	//Retour en phase d'attente
	geneTraj.angle.phase = ATTENTE;
	geneTraj.dist.phase = ATTENTE;

	//Arret boucle de position
	setEtatBouclePosition(OFF);

	//Les consignes sont nulles et l'on fixe le nouveau point de depart ici (on entre donc en regulation)
	geneTraj.angle.consigne=0;
	geneTraj.angle.consigneFiltree=0;
	geneTraj.dist.consigne=0;
	geneTraj.dist.consigneFiltree=0;
	fixation_position_depart();

	//Flag d'arrive a la consigne
	geneTraj.angle.fin = ARRIVE;
	geneTraj.dist.fin = ARRIVE;
}

/**
  * @use : Reinitialise les mesures pour effectuer un nouveau deplacement
  *
  * @param : void
  *
  * @retun : void
  *
  */
void fixation_position_depart()
{
    //RAZ des mesures
        asserv.dist.mesure = 0;
        asserv.angle.mesure = 0.0;

}

/**
  * @use : realise un calage du robot depuis la position de depart
  *
  * @param :
  *     - char: couleur = la couleur de notre equipe (pour savoir ou le robot se situe)
  *
  * @retun : 
  *		- char: error = le retour d'erreur (0 si tout est ok)
  *
  */
void calage_Position_Depart(char *error){
	unsigned int axeEssieu= getAxeEssieu();

		setAsservissementDistance(1);
		setAsservissementOrientation(1);

		rotation_De(18000,50,error);
		
		recule_Jusque_Blocage(error);
		setAngle(90);
		setYmm(axeEssieu);//Calage en X effectue
		
		avance_Avec_Freinage(170,50,OFF,error);
		rotation_De(-9000,50,error);
		
		recule_Jusque_Blocage(error);
		setAngle(0);
		setXmm(axeEssieu);//Calage en Y effectue
		
		//Postionnement configuration de depart
		avance_Avec_Freinage(150,50,OFF,error);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
	//Accesseurs //-----------------------------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************************************/
/****************************************************************************************************************/
/****************************************************************************************************************/
//Lecture

	/*TODO
	 *
	 *
	 */


/********************************************************************************************************************************************************************/
/********************************************************************************************************************************************************************/
//Ecriture

/****************************************************************************************************************/
/* FONCTION : void setVitesseDepMax(float vitesseMax)
 * Entrée : float vitesseMax	valeur max de la vitesse exprimée en pourcentage
 * Sortie : void
 * Utilitée: Permet de régler la vitesse max de déplacement
 */
void setVitesseDepMax(float vitesseMax)
{
	vitesseMax=fabs(vitesseMax);
	if(vitesseMax>100)	vitesseMax=100;
	vitesseMax=(geneTraj.dist.max.vitesse*vitesseMax)/100;
	//vitesseMax=(200*vitesseMax)/TickParmm;
	//GetResource(ResCommande);
	 geneTraj.dist.max.vitesse=vitesseMax;
	//ReleaseResource(ResCommande);
}

/****************************************************************************************************************/
/* FONCTION : void setVitesseAngleMax(float vitesseMax)
 * Entrée : float vitesseMax	valeur max de la vitesse angulaire exprimée en pourcentage
 * Sortie : void0x08000000
 * Utilitée: Permet de régler la vitesse max angulaire
 */
void setVitesseAngleMax(float vitesseMax)
{
	vitesseMax=fabs(vitesseMax);
	if(vitesseMax>100)	vitesseMax=100;
	vitesseMax=(geneTraj.angle.max.vitesse*vitesseMax)/100;
	//GetResource(ResCommande);
	 geneTraj.angle.max.vitesse=vitesseMax;
	//ReleaseResource(ResCommande);
}


/****************************************************************************************************************/
/* FONCTION : setConsigneDept()
 * Entrée : void
 * Sortie : void
 * Utilitée: Permet de donner une nouvelle consigne de déplacement
 */
void setConsigneDep(float consigneDep)
{
	//GetResource(ResCommande);
	geneTraj.dist.consigne=consigneDep; //nouvelle consigne de deplacement
	//ReleaseResource(ResCommande);
}

/****************************************************************************************************************/
/* FONCTION : setConsigneAngle()
 * Entrée : float consigneAngle en degré
 * Sortie : void
 * Utilitée: Permet de donner une nouvelle consigne d'orientation
 */
void setConsigneAngle(float consigneAngle)
{
	//GetResource(ResCommande);
	// geneTraj.angle.consigne=(consigneAngle*3.14159)/180;	//nouvelle consigne d'orientation traduite en radian
	geneTraj.angle.consigne = consigneAngle;	//nouvelle consigne d'orientation traduite en radian
	//ReleaseResource(ResCommande);
}


/****************************************************************************************************************/
/* FONCTION : void setPhaseDep(char phase)
 * Entrée : char phase	valeur de la phase de déplacement désirée
 * Sortie : void
 * Utilitée: Permet de changer la phase de déplacement en cours du générateur de trajectoire
 */
void setPhaseDep(char phase)
{
	//GetResource(ResCommande);
	geneTraj.dist.phase=phase;
	//ReleaseResource(ResCommande);
}

/****************************************************************************************************************/
/* FONCTION : void setPhaseAngle(char phase)
 * Entrée : char phase	valeur de la phase d'orientation désirée
 * Sortie : void
 * Utilitée: Permet de changer la phase d'orientation en cours du générateur de trajectoire
 */
void setPhaseAngle(char phase)
{
	//GetResource(ResCommande);
	geneTraj.angle.phase=phase;
	//ReleaseResource(ResCommande);
}

