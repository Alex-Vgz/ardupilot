/*
 * ims.h
 *
 *  Created on: Feb 15, 2017
 *      Author: Daniel Monier-Reyes
 */

// Utilisation de la HAL pour pouvoir écrire sur la console de monitoring
//#include "Copter.h"

// Pour les fonctions d'écriture/lecture de fichiers
#include <fstream>
#include <iostream>
#include <time.h>

#define LOG_TIME 600

/* Paramètres à changer selon le drone */

/*Paramètres du drone*/
#define COEF_POUSSEE 0.00000661               // coefficient de poussée trouvé sur la droite d'extrapolation de la force générée par un moteur en fonction de la rotation au carré (en kg.m/rad2)
#define COEF_TRAINEE 0.000000757
#define ENVERGURE 0.256 //en mètre

/*Paramètre moteurs*/
#define ROTATION_MAX 1393.3 //en rad/s



// -----------------------------------------------------------------------
// Déclaration des variables
// -----------------------------------------------------------------------

/* /!\ warnings lors de la compilation du programme /!\ 
 * utilisation de "static" dans ims.h car on utilise ses variables
 * plusieurs fois dans des fichiers .c différents ("control_ims1.c",
 * "control_ims2.c" et "control_ims3.c").
 * Cela permet d'éviter l'erreur "définition multiple".
 * A long terme, on gardera qu'un seul fichier control_ims.c donc 
 * on pourra remettre ces variables globales dans ce fichier.
 */

// Offset de calibration AHRS
static double offset_ahrs_roll;
static double offset_ahrs_pitch;
static double offset_ahrs_yaw;

// Paramètres du drone
static double b=COEF_POUSSEE;  	// Coefficient de poussée
static double d=COEF_TRAINEE;  	// Coefficient de trainée
static double l=ENVERGURE;    	// Envergure en mètre

// Valeurs de pwm min et max pour faire tourner les moteurs
static int16_t pwm_min,pwm_max;

// Index des moteurs (nécessaire pour adapter la numérotation des moteurs entre la loi de commande et Arducopter)
static const int16_t w1_index=4;                 // Représentation du drone en configuration en x
static const int16_t w2_index=2;                 //    W1   W3         4   1
static const int16_t w3_index=1;                 //       x      =>      x
static const int16_t w4_index=3;                 //    W4   W2         3   2

// Sortie des PIDs
static double u_theta, u_phi, u_r, u_z;

// Commandes (moteurs en rad/s)
static double w1,w2,w3,w4;

// Commandes (moteurs en pwm)
static int16_t w1_pwm,w2_pwm,w3_pwm,w4_pwm;

//fichieur ouvert
static bool fichier_log_ouvert=false;

// -----------------------------------------------------------------------
// Déclaration des fonctions
// -----------------------------------------------------------------------

void titre_log(std::ofstream *fichier,char nom_fichier[6]);
void ecriture_log(std::ofstream *fichier, double roll, double pitch, double yaw_rate ,double target_roll_rad, double target_pitch_rad, double target_yaw_rate_rad, double target_throttle_newton, double pos_theta, double pos_phi, double pos_r, double pos_z, double moteur1, double moteur2, double moteur3, double moteur4, int16_t moteur1_pwm, int16_t moteur2_pwm, int16_t moteur3_pwm, int16_t moteur4_pwm);
void test_pwm(int16_t* pwm_w1,int16_t* pwm_w2,int16_t* pwm_w3,int16_t* pwm_w4,int16_t max_pwm);

// -----------------------------------------------------------------------
// Déclaration des structures
// -----------------------------------------------------------------------

struct Coef_Correcteurs
{
    double xn;
    double xn_1;
    double xn_2;
    double yn_1;
    double yn_2;
};

// -----------------------------------------------------------------------
// Déclaration des classes
// -----------------------------------------------------------------------

// Classe permettant de récupérer les paramètres du drone comptenu dans un fichier
class Parametre_Drone
{
public:
    // Constructeur de la classe prenant en param�tre le nom du fichier contenant les paramètres
    Parametres_Drone(std::string nom_fichier);

    // Méthodes (fonctions)
    // Fonction permettant de regrouper les fonctions qui récupérent les données du fichier paramètre
    void set_parameters(void);
    // fonctions qui permettent de retourner les attributs de la classe "Parametre_Drone"
    float get_rotation_min(void) const;
    float get_rotation_max(void) const;
    float get_masse_arrachage(void) const;
    double get_coef_trainee(void) const;
    double get_coef_poussee(void) const;
    struct Coef_Correcteurs get_roulis(void) const;
    struct Coef_Correcteurs get_tangage(void) const;
    struct Coef_Correcteurs get_lacet(void) const;
    struct Coef_Correcteurs get_consigne_smooth(void) const;


private:
    // fonctions qui permettent de récupérer et initialiser les attributs de la classe "Parametres_Drone"
    void set_rotation_min(void);
    void set_rotation_max(void);
    void set_masse_arrachage(void);
    void set_coef_trainee(void);
    void set_coef_poussee(void);
    void set_roulis(void);
    void set_tangage(void);
    void set_lacet(void);
    void set_consigne_smooth(void);

    void aller_a_la_ligne(int num_ligne);
    void aller_a_la_ligne_apres(int num_ligne_apres);

    // Attributs (variables)
    float rotation_min, rotation_max;
    float masse_arrachage;
    double coef_trainee, coef_poussee;
    struct Coef_Correcteurs roulis,tangage,lacet,consigne_smooth;
    std::ifstream fichier;  // on ouvre le fichier en lecture;
};

// Classe représentant une equation récurrente du second ordre
class Correcteur_2nd_Ordre_Discret
{
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_2nd_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cxn_2=0,double m_cyn_1=0,double m_cyn_2=0)
    {
        cxn=m_cxn;
        cxn_1=m_cxn_1;
        cxn_2=m_cxn_2;
        cyn_1=m_cyn_1;
        cyn_2=m_cyn_2;

        xn=0;
        xn_1=0;
        xn_2=0;
        yn=0;
        yn_1=0;
        yn_2=0;

    }

    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn();

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);


private:
    double cxn,cxn_1,cxn_2,cyn_1,cyn_2;  // Coefficients de l'équation récurrente
    double xn;                           // Valeur de l'échantillon d'entrée x(n)
    double xn_1,xn_2;                    // Valeurs successives des échantillons d'entrée x(n-1), x(n-2)
    double yn;                           // Valeur de la sortie y(n)
    double yn_1,yn_2;                    // Valeurs successives de la sortie y(n-1), y(n-2)
};

class Correcteur_1er_Ordre_Discret
{
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_1er_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cyn_1=0)
    {
        cxn=m_cxn;
        cxn_1=m_cxn_1;
        cyn_1=m_cyn_1;

        xn=0;
        xn_1=0;
        yn=0;
        yn_1=0;

    }

    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn();

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);


private:
    double cxn,cxn_1,cyn_1;  // Coefficients de l'équation récurrente
    double xn;                          // Valeur de l'échantillon d'entrée x(n)
    double xn_1;                    	// Valeurs successives des échantillons d'entrée x(n-1)
    double yn;                          // Valeur de la sortie y(n)
    double yn_1;                    	// Valeurs successives de la sortie y(n-1)
};
