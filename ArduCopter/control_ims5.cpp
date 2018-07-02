/*
 * control_ims5.cpp
 *
 *  Created on: Jun 8, 2018
 *      Author: Daniel Monier-Reyes
 *
 *
 * Routines d'initialisation et d'appel pour le mode de vol IMS5
 * Ajout de la fonction smooth des consignes et création de la fonction titre_log(), reset_PID(),...
 * 
 */

#include "Copter.h"
#include "ims.h"
#include <time.h>

#define LOG_TIME 600

#define OFFSET_PWM 8.0                  // en % : valeur de pwm minimale envoyée au moteur
#define ANGLE_MAX_ROLL_PITCH 20.0       // angle maximal (en °) pour le pitch et le roll (par défaut, il est à 20°)
#define YAW_RATE_MAX 45.0               // vitesse angulaire maximal (en °/s) pour le yaw (par défaut, il est à 67°/s)
#define NOM_PROGRAMME "IMS5"
#define MASSE_DARRACHAGE 4.09           // masse maximale que les 4 moteurs fornissent (en kg)

// --------------------------------------------------------------------
//  Prototype des fonctions locales
// --------------------------------------------------------------------
void reset_PID(void);

// --------------------------------------------------------------------
// Déclaration des variables Globales
// --------------------------------------------------------------------

// classe qui récupére les paramètres du drone (attention à l'emplacement du fichier sur le drone)
Parametres_Drone params("/home/pi/ardupilot/ParametresDrone.conf");

// wu = 25 rad/s
// PID ROLL : y(n)=54.4335.x(n)+-107.3269.x(n-1)+52.9008.x(n-2)+1.839.y(n-1)+-0.83901.y(n-2)
Correcteur_2nd_Ordre_Discret pid_roll5(54.433526746576554,-107.3268778899634,52.900834170825107,1.839006063381362,-0.839006063381362);

// PID PITCH : y(n)=60.6081.x(n)+-119.5013.x(n-1)+58.9015.x(n-2)+1.839.y(n-1)+-0.83901.y(n-2)
Correcteur_2nd_Ordre_Discret pid_pitch5(60.608076049173278,-119.5012700088249,58.901525808112716,1.839006063381362,-0.839006063381362);

// PI R : y(n)=1.0955.x(n)+-1.0886.x(n-1)+1.y(n-1)
Correcteur_1er_Ordre_Discret pi_yaw5(1.095465982868761,-1.088640649642165,1);

// fonction smooth consigne
Correcteur_2nd_Ordre_Discret roll_smooth(0.00015242,0.00030483,0.00015242,1.9506,-0.95123),pitch_smooth(roll_smooth),yaw_rate_smooth(roll_smooth);

// ofstream est utilisé pour écrire un fichier CSV nommé IMS5_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
std::ofstream outf5;

// ---------------------------------------------------------------------------------------------
// ims5_init - Routine d'initialisation du mode de vol IMS5
// ---------------------------------------------------------------------------------------------
bool Copter::ims5_init(bool ignore_checks)
{
    int16_t offset_pwm; 
    // Initialisation des Offset AHRS
    //offset_ahrs_roll=ahrs.roll;
    //offset_ahrs_pitch=ahrs.pitch;
    //offset_ahrs_yaw=ahrs.get_gyro().z;
    
    // Récupération des valeurs min et max pwm pour la rotation des moteurs
    pwm_min = copter.motors.get_pwm_output_min();
    pwm_max = copter.motors.get_pwm_output_max();
    offset_pwm = OFFSET_PWM/100*(pwm_max - pwm_min);
    pwm_min = pwm_min + offset_pwm;

    params.set_parameters();

    // si le drone est atterri et que le mode à partir duquel nous passons n'a pas de throttle manuel et le manche des gaz est trop haut
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }

    // mise de l'altitude cible à zéro pour les rapports
    pos_control.set_alt_target(0);

    return true;
}

// ---------------------------------------------------------------------------------------------
// ims5_init - Routine d'appel du mode de vol IMS5 à exécuter à 400Hz
// ---------------------------------------------------------------------------------------------
void Copter::ims5_run()
{

    // --------------------------------------------------------------------
    // Déclaration des variables
    // --------------------------------------------------------------------

    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // Consignes
    double target_roll_rad, target_pitch_rad, target_yaw_rate_rad; // Target angles en radians et radians/s
    double target_throttle_newton;                                 // Target poussée en Newton
    double target_roll_smooth, target_pitch_smooth, target_yaw_rate_smooth; // Target angles en radians et radians/s en smooth

    // ------------------------------------------------------------------------
    // Programme d'origine permettant de récupérer les consignes
    // ------------------------------------------------------------------------

    // Récupération des consignes roulis et tangage (en centidegrés)
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, (double)ANGLE_MAX_ROLL_PITCH*100);

    // Récupération de la consigne en lacet (en centidegrés par seconde)
    target_yaw_rate = (double)YAW_RATE_MAX / 67.0 *  get_pilot_desired_yaw_rate(channel_yaw->get_control_in());  // 67.0 est la vitesse maximale par défaut du lacet

    // Récupération de la consigne en poussée (valeur entre 0 et 1)
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // ------------------------------------------------------------------------
    // Adaptation des valeurs des consignes Roll, Pitch, Yaw(R), Throttle en radian et radian/s
    // ------------------------------------------------------------------------

    // Conversion des consignes de la radiocommande Roll/Pitch de centi-degrés en radians
    target_roll_rad=double((target_roll*M_PI)/18000.0);
    target_pitch_rad=double((target_pitch*M_PI)/18000.0);
    // Conversion des consignes de la radiocommande YAW de centi-degrés par seconde en radians par seconde
    target_yaw_rate_rad=double((target_yaw_rate*M_PI)/18000.0);
    // Conversion des consignes de la radiocommande Throttle de centi-pourcentage en Newton
    target_throttle_newton=double(pilot_throttle_scaled*MASSE_DARRACHAGE*GRAVITY_MSS);

    // ------------------------------------------------------------------------
    // Spécifique Ardupilot - Gestion de l'armement des moteurs
    // ------------------------------------------------------------------------

    // Si les moteurs sont désarmés ou qu'ils ne tournent pas, ou encore, que le stick de la poussée est en bas
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);     // Tous les moteurs tournent lorsqu'ils sont armés
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt); // Réglage de la poussée et désactivation de la stabilisation

        // Reset des PIDs pour éviter qu'ils divergent
        reset_PID();

        if (fichier_log_ouvert==true) { // Si le fichier de log est ouvert alors le fermer
            outf5.close();
            fichier_log_ouvert=false;
        }

        return;
    } else // Sinon cela veut dire que les moteurs sont armés et que le drone est prêt à décoller
    {
        if (fichier_log_ouvert==false) { // Si le fichier de log n'est pas encore ouvert alors l'ouvrir
            // Fonction qui donne un titre au fichier
            titre_log(&outf5, NOM_PROGRAMME);
            fichier_log_ouvert=true;
        }
    }


    // Code d'origine - Réinitialisation du flag d'atterissage
    set_land_complete(false);

    // ------------------------------------------------------------------------
    // Calculs des PIDs avec offsets de compensation (car l'AHRS est calibrée à la main et donc
    // les valeurs des accéléromètres et gyroscopes ne sont jamais à 0 en position de repos)
    // Le x(n) de chaque PID prend la valeur calculée entre parenthèses et qui correspond à l'erreur
    // c'est à dire la valeur de la consigne moins la sortie de l'AHRS pour chaque axe concerné
    // L'opération cycle permet de décaler les valeurs successives de x(n) et y(n) dans le temps
    // ------------------------------------------------------------------------

    // Calcul de la consigne smooth
    // pour le roll
    roll_smooth.cycle(target_roll_rad);
    target_roll_smooth = roll_smooth.getyn();
    // pour le pitch
    pitch_smooth.cycle(target_pitch_rad);
    target_pitch_smooth = pitch_smooth.getyn();
    // pour le yaw rate
    yaw_rate_smooth.cycle(target_yaw_rate_rad);
    target_yaw_rate_smooth = yaw_rate_smooth.getyn();

    // Calcul PID Roll
    //pid_roll5.cycle(target_roll_smooth-(ahrs.roll-offset_ahrs_roll));
    // Calcul PID Pitch
    //pid_pitch5.cycle(target_pitch_smooth-(ahrs.pitch-offset_ahrs_pitch));
    // Calcul PID Yaw
    //pi_yaw5.cycle(target_yaw_rate_smooth-(ahrs.get_gyro().z-offset_ahrs_yaw));
    
    // Calcul PID Roll - Version sans prise en compte des offset de calibrage de l'AHRS au niveau du calcul de l'erreur
    pid_roll5.cycle(target_roll_smooth-ahrs.roll);
    // Calcul PID Pitch
    pid_pitch5.cycle(target_pitch_smooth-ahrs.pitch);
    // Calcul PID Yaw
    pi_yaw5.cycle(target_yaw_rate_smooth-ahrs.get_gyro().z);

    // Assignation des sorties de PIDs
    u_phi=pid_roll5.getyn();
    u_theta=pid_pitch5.getyn();
    u_r=pi_yaw5.getyn();
    //u_z=-target_throttle_newton/(cosf(ahrs.roll-offset_ahrs_roll)*cosf(ahrs.pitch-offset_ahrs_pitch));
    u_z=-target_throttle_newton/(cosf(ahrs.roll)*cosf(ahrs.pitch));

    // Calcul de la valeur des commandes
    w1=sqrt(constrain_float((d*u_phi+d*u_theta-b*l*u_r-d*l*u_z)/(b*d*l),0,2*ROTATION_MAX*ROTATION_MAX))/2;     //constrain_float(variable, valeur_min, valeur_max)
    w2=sqrt(constrain_float(-(d*u_phi+d*u_theta+b*l*u_r+d*l*u_z)/(b*d*l),0,2*ROTATION_MAX*ROTATION_MAX))/2;    // si la valeur de "variable" est inférieure à valeur_min alors la fonction retour : valeur_min
    w3=sqrt(constrain_float(-(d*u_phi-d*u_theta-b*l*u_r+d*l*u_z)/(b*d*l),0,2*ROTATION_MAX*ROTATION_MAX))/2;    // si la valeur de "variable" est supérieure à valeur_max alors la fonction retour : valeur_max
    w4=sqrt(constrain_float((d*u_phi-d*u_theta+b*l*u_r-d*l*u_z)/(b*d*l),0,2*ROTATION_MAX*ROTATION_MAX))/2;     // sinon elle retour la valeur de variable

    // Calcul des valeurs de PWM à envoyer à chaque moteur en fonction de w1, w2, w3, w4 - A commenter pour les tests de poussée
    w1_pwm=(w1/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min;
    w2_pwm=(w2/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min;
    w3_pwm=(w3/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min;
    w4_pwm=(w4/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min;

    // A décommenter pour les tests de poussée
    //w1_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w2_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w3_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w4_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;

    
    // --------------------------------------------------------------------
    // Test pwm des moteurs
    // --------------------------------------------------------------------

    // Test pour protection des moteurs
    test_pwm(&w1_pwm,&w2_pwm,&w3_pwm,&w4_pwm, pwm_max);

    // A mettre dans une zone de debug

    // ----------------------------------------------------------------------------------------
    // Affichage de la sortie de l'AHRS et des consignes
    // ----------------------------------------------------------------------------------------

    // Affichage des paramètres du drone
    hal.console->printf("Rotation min : %f et coef du roulis yn-1 : %f\n",params.get_rotation_min(),params.get_roulis().yn_1);

    // Affichage des consignes Roll, Pitch, Yaw, Throttle
    //hal.console->printf("Consignes - Roll: %f Pitch: %f Yaw: %f Throttle %f\n",target_roll_smooth*180/M_PI,target_pitch_smooth*180/M_PI,target_yaw_rate_smooth*180/M_PI, target_throttle_newton);

    // Affichage de l'erreur, des consignes et des sortie de l'AHRS
    //hal.console->printf("Erreur Roll : %f° = Consigne.Roll - AHRS.Roll = %f° - %f° \n",(target_roll_smooth - ahrs.roll)*180/M_PI, target_roll_smooth*180/M_PI, ahrs.roll*180/M_PI);
    //hal.console->printf("Erreur Pitch : %f° = Consigne.Pitch - AHRS.Pitch = %f° - %f° \n",(target_pitch_smooth - ahrs.pitch)*180/M_PI, target_pitch_smooth*180/M_PI, ahrs.pitch*180/M_PI);
    //hal.console->printf("Erreur Yaw : %f°/s = Consigne.Yaw - AHRS.Yaw = %f° - %f° \n",(target_yaw_rate_smooth - ahrs.get_gyro().z)*180/M_PI, target_yaw_rate_smooth*180/M_PI, ahrs.get_gyro().z*180/M_PI);
    
    // Affichage des sorties de l'AHRS
    //hal.console->printf("AHRS - Roll: %f Pitch:%f R:%f\n",ahrs.roll, ahrs.pitch, ahrs.get_gyro().z);

    // Affichage des valeurs PMW des moteurs
    //hal.console->printf("PWM - Min: %i Max: %i Actuel:%i w4:%i\n",pwm_min,pwm_max,pwm,w4_pwm);

    // Affichage des sorties des PIDs
    //hal.console->printf("PIDs - UPhi:%f, UTheta:%f, Ur:%f, Uz:%f\n",u_phi,u_theta,u_r,u_z);

    // Affichage des paramètres
    //hal.console->printf("Paramètres - d:%lf, b:%lf, l:%lf\n",d,b,l);

    // Affichage des commandes
    //hal.console->printf("Commandes - w1:%f, w2:%f, w3:%f, w4:%f\n",w1,w2,w3,w4);

    // --------------------------------------------------------------------
    // Ecriture des logs
    // --------------------------------------------------------------------

    // "AHRS.Roll,AHRS.Pitch,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,Uz,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm"

    ecriture_log(&outf5, ahrs.roll, ahrs.pitch, ahrs.get_gyro().z, target_roll_smooth, target_pitch_smooth, target_yaw_rate_smooth, target_throttle_newton, u_phi, u_theta, u_r, u_z, w1, w2, w3, w4, w1_pwm, w2_pwm, w3_pwm, w4_pwm);

    // ---------------------------------------------------------------------
    // Gestion Moteurs
    // ---------------------------------------------------------------------

    // Rotation des moteurs en fonction de la valeur en pwm des commandes
    motors.output_test(w1_index,w1_pwm);    // output_test : Fait tourner les moteurs à une valeur de PWM spécifiée : void output_test(uint8_t motor_seq, int16_t pwm);
    motors.output_test(w2_index,w2_pwm);    // motor_seq est le numéro du moteur (va de 1 au nombre de moteurs total)
    motors.output_test(w3_index,w3_pwm);    // pwm est la valeur pwm envoyée en sortie (normalement situé entre 1000 et 2000)
    motors.output_test(w4_index,w4_pwm);
}

// ---------------------------------------------------------------------
// Fonctions locales
// ---------------------------------------------------------------------

void reset_PID(void)
{
    pid_roll5.reset();
    pid_pitch5.reset();
    pi_yaw5.reset();
    roll_smooth.reset();
    pitch_smooth.reset();
    yaw_rate_smooth.reset();
}
