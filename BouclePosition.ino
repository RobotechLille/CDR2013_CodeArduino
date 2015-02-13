//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waBouclePosition, 64);

// Declare thread function for BouclePosition (priority 34).
NIL_THREAD(BouclePosition, arg) {
  while (TRUE) {
    //Attente du signal  de réveil
    nilSemWait(&wakeBouclePosition);  
    
    //Algo
    if(bouclePosition.etat == ON && (geneTraj.dist.phase == ACCELERATION || geneTraj.dist.phase == CONSTANTE))
    {
        //Cas d'une marche avant
        if(bouclePosition.sensDeplacement == AVANCE)
        {
            //La nouvelle consigne en distance correspond a la somme de la distance parcouru 
                // et de la distance restance jusqu'au point voulu
            geneTraj.dist.consigne = (float)((asserv.dist.mesure/TickParmm) + rechercheDistanceConsigne(bouclePosition.objectif.x, bouclePosition.objectif.y));
            //La nouvelle consigne en angle est calculee a partir de la position actuelle
            geneTraj.angle.consigne = (float)(rechercheAngleConsigne(bouclePosition.objectif.x, bouclePosition.objectif.y));
        }
        //Cas d'une marche arrière
        else
        {
            float consigneAngle=0;

            //La nouvelle consigne en distance correspond a la somme de la distance parcouru 
                // et de la distance restance jusqu'au point voulu
            geneTraj.dist.consigne = (float)((asserv.dist.mesure/TickParmm) + rechercheDistanceConsigne(bouclePosition.objectif.x, bouclePosition.objectif.y));
            //La nouvelle consigne en angle est calculee a partir de la position actuelle
            consigneAngle = (float)(rechercheAngleConsigne(bouclePosition.objectif.x,bouclePosition.objectif.y));
            consigneAngle = consigneAngle + constPI;
            if(consigneAngle > constPI) consigneAngle = consigneAngle - 2*constPI;

            geneTraj.angle.consigne = consigneAngle;
        }	
    }
    
    
    //Affichage Ordonnancement
    #if (AFFICHAGE == ON && AFFICHAGE_ORDONNANCEMENT == ON)
    Serial.print("!");  
    #endif
  }  
}

/* FONCTIONS ANNEXES */
/**
  * @use : accesseur en lecture pour bouclePosition.etat
  *
  * @param : void
  *
  * @retun : 
  *     - byte: bouclePosition.etat = l'etat de la boucle, actif ou non (ON/OFF)
  *
  */
byte getEtatBouclePosition(void)
{
 	return bouclePosition.etat;
}


/**
  * @use : accesseur en lecture pour bouclePosition.objectif.x
  *
  * @param : void
  *
  * @retun : 
  *     - unsigned long: bouclePosition.objectif.x = l'abscisse du point que l'on cherche a atteindre en mm
  *
  */
unsigned long getXvoulu(void)
{
 	return bouclePosition.objectif.x;
}


/**
  * @use : accesseur en lecture pour bouclePosition.objectif.y
  *
  * @param : void
  *
  * @retun : 
  *     - unsigned long: bouclePosition.objectif.y = l'ordonnee du point que l'on cherche a atteindre en mm
  *
  */
unsigned long getYvoulu(void)
{
 	return bouclePosition.objectif.y;
}

/********************************************************************************************************************************************************************/
/********************************************************************************************************************************************************************/
//Ecriture


/**
  * @use : accesseur en ecriture pour bouclePosition.etat
  *
  * @param :
  *     - byte: etat = l'etat de la boucle, actif ou non (ON/OFF)
  *
  * @retun : void
  *
  */
void setEtatBouclePosition(byte etat)
{
 	bouclePosition.etat = etat;
}


/**
  * @use : accesseur en ecriture pour bouclePosition.sensDeplacement
  *
  * @param :
  *     - byte: sens = sens du deplacement voulu (AVANCE/RECULE)
  *
  * @retun : void
  *
  */
void setSensDeplacement(byte sens)
{
    bouclePosition.sensDeplacement = sens;
}


/**
  * @use : accesseur en ecriture pour bouclePosition.objectif.x
  *
  * @param :
  *     - unsigned long: coordX = l'abscisse du point que l'on cherche a atteindre en mm
  *
  * @retun : void
  *
  */
void setXvoulu(unsigned long coordX)
{
 	bouclePosition.objectif.x = coordX;
}


/**
  * @use : accesseur en ecriture pour bouclePosition.objectif.y
  *
  * @param :
	*     - unsigned long: coordY = l'ordonnee du point que l'on cherche a atteindre en mm
  *
  * @retun : void
  *
  */
void setYvoulu(unsigned long coordY)
{
 	bouclePosition.objectif.y = coordY;
}
