#include "Copter.h"
#include "ims.h"
#include <time.h>

#define LOG_TIME 600


/*
 * Init and run calls for IMS1 flight mode
 */

// Créer une fonction de chargement de paramètres depuis un fichier généré par Matlab, et la mettre dans Init du mode de vol
    // Objets Correcteurs PID pour des moteurs Protronik Kv750 et une poussée de 2,711Kg

    // PID ROLL : y(n)=14.6972062.x(n)-29.2788573.x(n-1)+14.5818480.x(n-2)+1.8902053.y(n-1)-0.8902053.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_roll(14.6972062,-29.2788573,14.5818480,1.8902053,-0.8902053);

    // PID PITCH : y(n)=74.4323381.x(n)-148.6091737.x(n-1)+74.1770095.x(n-2)+1.4994958.y(n-1)-0.4994958.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_pitch(74.4323381,-148.6091737,74.1770095,1.4994958,-0.4994958);

    // PID R : y(n)=0.2856231.x(n)-0.5596865.x(n-1)+0.2740904.x(n-2)+1.9839837.y(n-1)-0.9839837.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_yaw(0.2856231,-0.5596865,0.2740904,1.9839837,-0.9839837);

    // Offset de calibration AHRS
    double offset_ahrs_roll;
    double offset_ahrs_pitch;
    double offset_ahrs_yaw;

    // Paramètres du drone
    double b=0.0000069245;  // Coefficient de poussée
    double d=0.000000757;   // Coefficient de trainée
    double l=0.256;         // Envergure en mètre

    // Valeurs de pwm min et max pour faire tourner les moteurs
    int16_t pwm_min,pwm_max;

    // Index des moteurs (nécessaire pour adapter la numérotation des moteurs entre la loi de commande et Arducopter)
    int16_t w1_index=4;
    int16_t w2_index=2;
    int16_t w3_index=1;
    int16_t w4_index=3;

    // Sortie des PIDs
    double u_theta, u_phi, u_r, u_z;

    // Commandes (moteurs en rad/s)
    double w1,w2,w3,w4;

    // Commandes (moteurs en pwm)
    int16_t w1_pwm,w2_pwm,w3_pwm,w4_pwm;

    // ofstream est utilisé pour écrire un fichier CSV nommé IMS1_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
    std::ofstream outf;
    bool fichier_log_ouvert=false;
    //long csv_timer=0;

    // Calcul de la période réelle de la boucle soit disant à 100Hz
    //time_t timer_start,timer_stop;


// ims1_init - initialise IMS1 controller
bool Copter::ims1_init(bool ignore_checks)
{
    // Offset AHRS
    offset_ahrs_roll=ahrs.roll;
    offset_ahrs_pitch=ahrs.pitch;
    offset_ahrs_yaw=ahrs.get_gyro().z;

    // Récupération des valeurs min et max pwm pour la rotation des moteurs
    pwm_min=copter.motors.get_pwm_output_min();
    pwm_max=copter.motors.get_pwm_output_max();

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// ims1_run - runs the main IMS1 controller
// should be called at 100hz or more
void Copter::ims1_run()
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

    // ------------------------------------------------------------------------
    // Programme d'origine permettant de récupérer les consignes
    // ------------------------------------------------------------------------

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    // TEMP
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    // TEMP
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // ------------------------------------------------------------------------
    // Adaptation des valeurs des consignes Roll, Pitch, Yaw(R), Throttle en radian et radian/s
    // ------------------------------------------------------------------------

    // Conversion des consignes de la radiocommande Roll/Pitch de centi-degrés en radians
    target_roll_rad=double((target_roll*M_PI)/18000);
    target_pitch_rad=double((target_pitch*M_PI)/18000);
    // Conversion des consignes de la radiocommande YAW de centi-degrés par seconde en radians par seconde
    target_yaw_rate_rad=double((target_yaw_rate*M_PI)/18000);
    // Conversion des consignes de la radiocommande Throttle de centi-pourcentage en Newton
    target_throttle_newton=double(pilot_throttle_scaled*2.95*GRAVITY_MSS);


    // ----------------------------------------------------------------------------------------
    // Affichage de la sortie de l'AHRS et des consignes
    // ----------------------------------------------------------------------------------------

    // Affichage des consignes Roll, Pitch, Yaw, Throttle
    //hal.console->printf("Consignes - Roll: %f Pitch: %f Yaw: %f Throttle %f\n",target_roll_rad,target_pitch_rad,target_yaw_rate_rad, target_throttle_newton);

    // Affichage des sorties de l'AHRS
    //hal.console->printf("AHRS - Roll: %f Pitch:%f R:%f\n",ahrs.roll, ahrs.pitch, ahrs.get_gyro().z);

    // ------------------------------------------------------------------------
    // Spécifique Ardupilot - Code d'origine pour gérer l'armement des moteurs
    // ------------------------------------------------------------------------

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // Reset des PIDs pour éviter qu'ils divergent
        pid_roll.reset();
        pid_pitch.reset();
        pid_yaw.reset();

        if (fichier_log_ouvert==true) { // Si le fichier de log est ouvert alors le fermer
            outf.close();
            fichier_log_ouvert=false;
        }

        return;
    } else // Sinon cela veut dire que les moteurs sont armés et que le drone est prêt à décoller
    {
        if (fichier_log_ouvert==false) { // Si le fichier de log n'est pas encore ouvert alors l'ouvrir

            // Construction de la date et de l'heure pour rendre le nom du fichier de log unique
            time_t rawtime;
            struct tm * timeinfo;
            char buffer [30];
            char log_file_name [80];

            time (&rawtime);
            timeinfo = localtime (&rawtime);

            strftime (buffer,30,"%F--%H-%M-%S",timeinfo);

            strcpy(log_file_name,"/home/pi/ardupilot/build/navio2/bin/IMS1_CSV_LOG-");
            strcat(log_file_name,buffer);
            strcat(log_file_name,".dat");

            outf.open(log_file_name); // Création d'un fichier de log unique
            // Ecriture d'une entête pour savoir à quoi correspond les données
            outf << "AHRS.Roll,AHRS.Pitch,AHRS.Yaw,AHRS.P,AHRS.Q,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm,pilot_throttle_scaled" << std::endl;

            fichier_log_ouvert=true;
        }
    }

    // ------------------------------------------------------------------------
    // Réinitialisation du flag d'atterissage
    // ------------------------------------------------------------------------
    set_land_complete(false);

    // ------------------------------------------------------------------------
    // Calculs des PIDs avec offsets de compensation (car l'AHRS est calibrée à la main et donc
    // les valeurs des accéléromètres et gyroscopes ne sont jamais à 0 en position de repos)
    // Le x(n) de chaque PID prend la valeur calculée entre parenthèses et qui correspond à l'erreur
    // c'est à dire la valeur de la consigne moins la sortie de l'AHRS pour chaque axe concerné
    // L'opération cycle permet de décaler les valeurs successives de x(n) et y(n) dans le temps
    // ------------------------------------------------------------------------

    // Calcul PID Roll
    pid_roll.cycle(target_roll_rad-ahrs.roll+offset_ahrs_roll);
    // Calcul PID Pitch
    pid_pitch.cycle(target_pitch_rad-ahrs.pitch+offset_ahrs_pitch);
    // Calcul PID Yaw
    pid_yaw.cycle(target_yaw_rate_rad-ahrs.get_gyro().z+offset_ahrs_yaw);

    // Calcul PID Roll
    //pid_roll.cycle(target_roll_rad-ahrs.roll);
    // Calcul PID Pitch
    //pid_pitch.cycle(target_pitch_rad-ahrs.pitch);
    // Calcul PID Yaw
    //pid_yaw.cycle(target_yaw_rate_rad-ahrs.get_gyro().z);

    // Assignation des sorties de PIDs
    u_phi=pid_roll.getyn();
    u_theta=pid_pitch.getyn();
    u_r=pid_yaw.getyn();
    u_z=-target_throttle_newton/(cosf(ahrs.roll)*cosf(ahrs.pitch));

    // Calcul de la valeur des commandes
    w1=sqrt((d*u_phi+d*u_theta-b*l*u_r-d*l*u_z)/(b*d*l))/2;
    w2=sqrt(-(d*u_phi+d*u_theta+b*l*u_r+d*l*u_z)/(b*d*l))/2;
    w3=sqrt(-(d*u_phi-d*u_theta-b*l*u_r+d*l*u_z)/(b*d*l))/2;
    w4=sqrt((d*u_phi-d*u_theta+b*l*u_r-d*l*u_z)/(b*d*l))/2;

    // Calcul des valeurs de PWM à envoyer à chaque moteur en fonction de w1, w2, w3, w4
    w1_pwm=((w1*(pwm_max-pwm_min))/1393.3)+pwm_min;
    w2_pwm=((w2*(pwm_max-pwm_min))/1393.3)+pwm_min;
    w3_pwm=((w3*(pwm_max-pwm_min))/1393.3)+pwm_min;
    w4_pwm=((w4*(pwm_max-pwm_min))/1393.3)+pwm_min;

    //w1_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w2_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w3_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w4_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;

    // A mettre dans une zone de debug

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

    // Horodatage du début
    //time(&timer_start);
    //outf << ctime(&timer_start);
    //outf << std::endl;

    // A mettre dans une fonction
    // "AHRS.Roll,AHRS.Pitch,AHRS.Yaw,AHRS.P,AHRS.Q,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm,pilot_throttle_scaled"

        outf << ahrs.roll;
        outf << ",";
        outf << ahrs.pitch;
        outf << ",";
        outf << ahrs.yaw;
        outf << ",";
        outf << ahrs.get_gyro().x;
        outf << ",";
        outf << ahrs.get_gyro().y;
        outf << ",";
        outf << ahrs.get_gyro().z;
        outf << ",";
        outf << target_roll_rad;
        outf << ",";
        outf << target_pitch_rad;
        outf << ",";
        outf << target_yaw_rate_rad;
        outf << ",";
        outf << target_throttle_newton;
        outf << ",";
        outf << pid_roll.getyn();
        outf << ",";
        outf << pid_pitch.getyn();
        outf << ",";
        outf << pid_yaw.getyn();
        outf << ",";
        outf << w1;
        outf << ",";
        outf << w2;
        outf << ",";
        outf << w3;
        outf << ",";
        outf << w4;
        outf << ",";
        outf << w1_pwm;
        outf << ",";
        outf << w2_pwm;
        outf << ",";
        outf << w3_pwm;
        outf << ",";
        outf << w4_pwm;
        outf << ",";
        outf << pilot_throttle_scaled;
        outf << std::endl;

    // ---------------------------------------------------------------------
    // Gestion Moteurs
    // ---------------------------------------------------------------------

    // Rotation des moteurs en fonction de la valeur en pwm des commandes
    motors.output_test(w1_index,w1_pwm);
    motors.output_test(w2_index,w2_pwm);
    motors.output_test(w3_index,w3_pwm);
    motors.output_test(w4_index,w4_pwm);





}
