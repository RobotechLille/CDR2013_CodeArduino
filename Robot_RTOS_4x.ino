/* Programme principal pilotant le robot */
// Ce fichier contient les entetes tandis-que le programme principal se trouve dans Z_MAIN

//------------------------------------------------------------------------------
/* INCLUDES */
  //Système temps réel
  #include <NilRTOS.h>
  
  // Use tiny unbuffered NilRTOS NilSerial library.
  #include <NilSerial.h>
  // Macro to redefine Serial as NilSerial to save RAM.
  // Remove definition to use standard Arduino Serial.
  #define Serial NilSerial
  
  //Timer pour l'ordonnanceur
  #include <NilTimer1.h>
  
  //Pilotages des moteurs à CC
  #include <DriverMoteur.h>
  
  //Lecture des encodeurs
  #define OPTIMIZE_INTERUPTS  //Optimise la lecture des encodeurs
  #include <Encoder.h>
  
  //Detection d'obstacles à l'aide de sonars Ultrasons de types SRF05
  #include <SRF05.h>
  
//------------------------------------------------------------------------------
/* DEFINES */
/* Général */
  #define ON                                    1
  #define OFF                                   0
  
  #define AFFICHAGE                             ON    //Indique si l'on fait un affichage ou non
  //Précise le type d'affichage
  #define AFFICHAGE_ORDONNANCEMENT              OFF
  #define AFFICHAGE_STACKUSE                    OFF
  #define AFFICHAGE_STANDARD                    OFF
  
  //Constante Pi
  #define constPI 3.14159265
  
  //Pins de connections
  #define PIN_INT_ROUGE   16
  #define PIN_INT_BLEU    17
  #define PIN_JACK_DEPART 53
    
/*Constantes d'odometrie*/
  //Couleurs de départs
  #define ROUGE               0
  #define BLEU                1
  //Nombre de ticks par mm
  #define TickParmm           5.4      //5,949
  //Ticks entre les deux roues codeuses en mm
  #define LongueurEntreAxemm    125    //A mettre à jour
  #define LongueurEntreAxe      (LongueurEntreAxemm*TickParmm)
  //Distance centre essieu -> bord du robot en mm
	// /!\ A Ajuster
  #define CentreEssieu          235
  //Distance axe essieu -> cul du robot (bille de calage) en mm
  #define AxeEssieu             108
  
/*Constantes de Génération de Trajectoire*/
    #define TpsEchGeneTraj 20
    //Phases de l'algorithme d'etat
    #define ATTENTE      0
    #define ACCELERATION 1
    #define CONSTANTE    2
    #define DECELERATION 3
    #define RECUPERATION 4

    //Flags
    #define EN_COURS     0
    #define ARRIVE       1
    
/*Constante d'Asservissement*/
    #define TpsEchCommande 5
    
    /* ------ PID Distance ------ */
    //Phase de déplacement
    #define KP_D_Dep        (0.029)                    // CDR2012 (2.9            /104)
    #define KI_D_Dep        (KP_D_Dep*0.01)             //((KP_D_Dep*104)*0.0   /104)
    #define KD_D_Dep        (KP_D_Dep*104)            //((KP_D_Dep*104)*1.75  /104)
    //NB : ne se règle pas car pris en compte par la génération de trajectoire
    //Phase d'attente et de récupération de l'écart
    #define KP_D_Att        (1.5)            //0.71          //(400.0          /104)            //100  
    #define KI_D_Att        (KP_D_Att*0.0) //0.001           //((KP_D_Att*104)*0.01  /104)
    #define KD_D_Att        (KP_D_Att*0.0)            //((KP_D_Att*104)*0.1  /104)     //0.067
    //NB : se règle avec la méthode de la réponse graduée voulue (voir avec Valentin VERGEZ)

    /* ------ PID Orientation ------ */
    //Phase de déplacement
    #define KP_O_Dep       (8.0)                      //(8.0)  //8
    #define KI_O_Dep       (KP_O_Dep*0.0)             //(KP_O_Dep*0.0000625)
    #define KD_O_Dep       (KP_O_Dep*0.0)             //(KP_O_Dep*0)
    //NB : ne se règle pas car pris en compte par la génération de trajectoire
    //Phase d'attente et de récupération de l'écart
    #define KP_O_Att       (7000)            //3822         //(7.0) //7 //700  //2000
    #define KI_O_Att       (KP_O_Att*0.0)            //(KP_O_Att*0.0)
    #define KD_O_Att       (KP_O_Att*0.0)            //(KP_O_Att*1.5)     //2.3
    //NB : se règle avec la méthode de la réponse graduée voulue (voir avec Valentin VERGEZ)    
    
/*Constantes de la boucle de position*/
    #define AVANCE  1
    #define RECULE  2
    
    //DEFINES d'arret
    #define MARCHE         0   
    #define STOP           1
    #define ARRET          2

//------------------------------------------------------------------------------
/* TYPES */ //Types et stuctures de données utiles
/*DIVERS*/
  //Distances avant et arrière, droite et gauches mesurées par les sonars
  typedef struct
  {
    int fd,fg,rd,rg;
  }Dist;

  //Coordonnées du robot en mm et angle en dixieme de degrés et valeurs des encodeurs
  typedef struct
  {
    long x,y,a;         //mm, mm, dixieme de degrés
    float xtick,ytick;  //coordonnées en ticks codeurs
    float arad;        //angle en rad
    long encD, encG;   //valeurs des encodeurs
    struct {long encD,encG;}prec;   //Sauvegardes
  }Position;

/*ASSERVISSEMENT*/
  typedef struct 
  {
      byte etat; 
      float ecart, sommeEcart, vitesse, commande, mesure, variation;
  }utilsAsserv;

  typedef struct
  {
    unsigned long iteration;
    utilsAsserv dist;
    utilsAsserv angle;
    struct {
      signed int motD, motG;
      struct { unsigned int motD, motG;} abs;
      float correcteur;
    }commande;
    unsigned int maxCommandeMoteur;
  }VG_Asservissement;

/*BOUCLE POSITION*/
  typedef struct
  {
    struct {
      unsigned long x,y;
      float a;
    }objectif;
    byte etat;
    byte sensDeplacement;
  }VG_BouclePosition;

/*GENERATEUR TRAJECTOIRE*/
typedef struct {
  float consigne, consigneFiltree;
  byte phase, fin;
  struct {
    float vitesse, consigne;
    unsigned int tpsAcceleration, tpsDeceleration;
  }max;
}utilsGeneTraj;

typedef struct {
  utilsGeneTraj dist;
  utilsGeneTraj angle;
  byte arretForce;
}VG_GeneTraj;

/*ODOMETRIE*/
  typedef struct {
    unsigned long iteration;
    struct {
      float dist, angle, gauche, droit;
    }delta;
  }VG_Odometrie;

//------------------------------------------------------------------------------
/* OBJETS */ //Instances de classes prédéfinies
DriverMoteur mG(5,6,7);    //Moteur droit sur pins 5,6,7 avec PWM sur 7
DriverMoteur mD(8,9,10);   //Moteur gauche sur pins 8,9,10 avec PWM sur 10

Encoder encD(18,19);       //Encodeur droit avec A->19 et B->18 (impératif, doit etre sur des entrées d'interruptions)
Encoder encG(20,21);       //Encodeur gauche avec A sur 20 et B sur 21

SRF05 usFG(37);            // 4 sonars à UltraSons sur les pins 31, 33, 35 et 37
SRF05 usFD(38);            // Avant droit = Front Droit = FD, idem : RearGauche etc
SRF05 usRG(43);
SRF05 usRD(42);


//------------------------------------------------------------------------------
/* VARIABLES GLOBALES */ //Variables utilisées dans tout le programme
//Coefficient de détection d'adversaire
float detectionObstacle;
float coeffObstacle;

//Informations robots
Dist dist;                 //Les distances avant obstacles
Position pos;              //Informations de positions

//Gestion du temps de match
boolean MATCH_EN_COURS = false;
boolean FIN_MATCH = false;

//Gestion de la position de départ
byte CouleurDepart = ROUGE;

//Gestion d'erreurs
byte _error;



/*Asservissement*/
  VG_Asservissement asserv;
  void initAsservissement()
  {
    asserv.dist.etat = asserv.angle.etat             = OFF;
    asserv.dist.ecart = asserv.angle.ecart           = 0.0; //en ticks et radians
    asserv.dist.sommeEcart = asserv.angle.sommeEcart = 0.0;
    asserv.dist.vitesse = asserv.angle.vitesse       = 0.0; //en ticks/s et rad/s
    asserv.dist.commande = asserv.angle.commande     = 0.0; //en %0 (pour 1000), commande moteur
    asserv.dist.mesure = asserv.angle.mesure         = 0.0; //en ticks et radians
    asserv.dist.variation = asserv.angle.variation   = 0.0; //en ticks et radians

    asserv.iteration                                    = 0; //nombre d'execution de la tâche
    asserv.maxCommandeMoteur                            = 1000;

    asserv.commande.motD = asserv.commande.motG         = 0; //en %0 (pour 1000), commande moteur
    asserv.commande.abs.motD = asserv.commande.abs.motG = 0;
    asserv.commande.correcteur                          = 0.0; //coefficient de correction des commandes en cas de saturation
  }

/*Boucle position*/
  VG_BouclePosition bouclePosition;
  void initBouclePosition()
  {
    bouclePosition.objectif.x = bouclePosition.objectif.y = 0;
    bouclePosition.objectif.a                             = 0.0;
    bouclePosition.etat                                   = OFF;
    bouclePosition.sensDeplacement                        = AVANCE;
  }

/*Odométrie*/
  VG_Odometrie odo;
  void initOdometrie()
  {
    //Variables internes à l'odométrie
    odo.iteration = 0;
    odo.delta.droit  = 0;
    odo.delta.gauche = 0;
    odo.delta.dist = 0;
    odo.delta.angle = 0;
    
    //Encodeurs
    pos.encD=0;
    pos.encG=0;
    pos.prec.encD=0;
    pos.prec.encG=0;
    
    //Position
    pos.x = 0;
    pos.y = 0;
    pos.a = 0;
    pos.xtick = 0;
    pos.ytick = 0;
    pos.arad = 0;    
  }

/*Generation de trajectoire*/
  VG_GeneTraj geneTraj;
  void initGeneTraj()
  {
    geneTraj.dist.consigne = geneTraj.angle.consigne                       = 0.0;
    geneTraj.dist.consigneFiltree = geneTraj.angle.consigneFiltree         = 0.0;
    geneTraj.dist.phase = geneTraj.angle.phase                             = ATTENTE;
    geneTraj.dist.fin = geneTraj.angle.fin                                 = true;
    geneTraj.dist.max.tpsDeceleration = geneTraj.angle.max.tpsDeceleration = 0.0;
    geneTraj.dist.max.consigne = geneTraj.angle.max.consigne               = 0.0;

    geneTraj.arretForce = false;

    geneTraj.dist.max.vitesse          = 10000 / KP_D_Dep;
    geneTraj.angle.max.vitesse         = 10000 / KP_O_Dep;
    geneTraj.dist.max.tpsAcceleration  = 400;
    geneTraj.angle.max.tpsAcceleration = 250;
  }

  float saveSeuil,
      saveCV,
      saveRap,
      saveVit;

//------------------------------------------------------------------------------
/* SETUP */ //S'execute une fois et permet d'effectuer les initialisations
void setup() {
  //Serial
  Serial.begin(9600);
  
  //Initialisations des taches
  initAsservissement();
  initBouclePosition();
  initGeneTraj();
  initOdometrie();
  
  //Augmentation de la fréquence PWM (30kHz)
    //Valable sur les pins 2 et 3 ; 5,6,7,8,9 et 10
    //Les pins 4 et 13 tournent à 980Hz à cause du Timer0 déjà utilisé
    //Les pins 11 et 12 sont désactivées à cause du Timer1
  TCCR2B = _BV(CS20);
  TCCR3B = _BV(CS30);
  TCCR4B = _BV(CS40);
  
  //Others 
  //for(int i = 1; i<= 13; i++)  analogWrite(i, 127);  //Test des 13 PWM en raport cyclique 50%
  
  //Entrées de PULL-UP
  pinMode(PIN_INT_BLEU,INPUT);
  digitalWrite(PIN_INT_BLEU,HIGH);
  pinMode(PIN_INT_ROUGE,INPUT);
  digitalWrite(PIN_INT_ROUGE,HIGH);
  pinMode(PIN_JACK_DEPART,INPUT);
  digitalWrite(PIN_JACK_DEPART,HIGH);
  
  //Activation de la détection d'adversaire
  detectionObstacle = true;  
  
  // Start kernel
  nilSysBegin();
  
  asserv.dist.etat=OFF;
}


/* SEMAPHORES */ // Permettent d'activer un thread
SEMAPHORE_DECL(wakeAsservissement,0);
SEMAPHORE_DECL(wakeBouclePosition,0);
SEMAPHORE_DECL(wakeCapteur,0);
SEMAPHORE_DECL(wakeGenerateurTrajectoire,0);
SEMAPHORE_DECL(wakeGestionTemps,0);
SEMAPHORE_DECL(wakeIntelligence,0);
SEMAPHORE_DECL(wakeOdometrie,0);



//------------------------------------------------------------------------------
/* PROTOTYPES */
  /*Générateur de trajectoire*/  
  void avance_Avec_Freinage(float distance, float vitesse, char PositionLoop, char*error);
  void recule_Avec_Freinage(float distance, float vitesse, char PositionLoop, char*error);
  void recule_Jusque_Blocage(char*error);
  void rotation_De(float angle, int vitesse, char*error);
  void orientation_De(float angle_final, int vitesse, char *error);
      //Fonctions de recherche de consignes avec coordonnees//------------------
  /****************************************************************************/
  int rechercheDistanceConsigne(int coordX, int coordY);
  int rechercheAngleConsigne(float coordX, float coordY);
      //Fonctions de deplacement avec coordonnees//----------------------------
  /****************************************************************************/
  void rotation_Vers_Le_Pt(int coordX,int coordY,int vitesse, char*error);
  void rotation_Arriere_Vers_Le_Pt(int coordX, int coordY,int vitesse, char*error);
  void avance_Rectiligne_Avec_Freinage_Vers_Le_Pt(int coordX, int coordY, int vitesse, char*error);
  void avance_Rectiligne_En_Marche_Arriere_Avec_Freinage_Vers_Le_Pt(int coordX, int coordY, int vitesse, char*error);
      //Fonctions de calage et d'initialisation d'un point//--------------------
  /****************************************************************************/
  void arretDeplacement();
  void fixation_position_depart();
  void calage_Position_Depart(char *error);
      //Accesseurs //-----------------------------------------------------------
  /****************************************************************************/
  //Lecture
  
      /*TODO
       *
       *
       */
       
  /****************************************************************************/
  //Ecriture
  
  void setVitesseDepMax(float vitesseMax);
  void setVitesseAngleMax(float vitesseMax);
  void setConsigneDep(float consigneDep);
  void setConsigneAngle(float consigneAngle);
  void setPhaseDep(char phase);
  void setPhaseAngle(char phase);




  /*Odométrie*/
  void resetOdometrie(int x = 0, int y = 0, int a = 0);
  int getX();
  float getY();
  int getAngle();
  char getTickParmm();
  unsigned int getLongueurEntreAxe();
  void setXmm(unsigned int x);
  void setYmm(unsigned int y);
  void setAngle(float angledegree);
  unsigned int getAxeEssieu();
  unsigned int getCentreEssieu();
  
  /*Asservissement*/
  //GESTION DES ASSERVISSEMENTS
float PID_Distance(float ecart, float vitesse); 	//regulateur PID numerique pour l'asservissement en distance
float PID_Angle(float ecart, float vitesse); 	 	//regulateur PID numerique pour l'asservissement en angle
void setConsigneDep(float consigneDep); 				//ecrit une nouvelle consigne de deplacement
void setConsigneAngle(float consigneAngle); 				//ecrit une nouvelle consigne d'orientation en degré
void setAsservissementDistance( char asservissementDistance); 		//active ou desactive l'asservissement en distance
void setAsservissementOrientation( char asservissementOrientation); 	//active ou desactive l'asservissement en orientation
void init_PID(void); //réinitialise les PID


//ACCESSEUR DE L'ETAT DE L'ECART
int getEcartDistance();	//Permet d'obtenir la valeur de l'ecart en distance en entier
int getEcartAngle();	//Permet d'obtenir la valeur de l'ecart en angle en entier


/* Boucle Position */
	//Accesseurs //-----------------------------------------------------------------------------------------------
//Lecture
unsigned char getEtatBouclePosition(void);
unsigned long getXvoulu(void);
unsigned long getYvoulu(void);
/********************************************************************************************************************************************************************/
//Ecriture
void setEtatBouclePosition(unsigned char etat);
void setSensDeplacement(unsigned char sens);
void setXvoulu(unsigned long coordX);
void setYvoulu(unsigned long coordX);
