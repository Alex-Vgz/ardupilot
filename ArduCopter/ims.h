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
#define COEF_POUSSEE 0.0000069245
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
// Déclaration des classes
// -----------------------------------------------------------------------

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

