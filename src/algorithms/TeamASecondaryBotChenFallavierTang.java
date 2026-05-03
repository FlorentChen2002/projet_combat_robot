package algorithms;

import java.util.ArrayList;

import characteristics.IFrontSensorResult;
import characteristics.Parameters;

/**
 * PatientHunterSecondary / TeamASecondaryBotChenFallavierTang
 *
 * Phase 1 (moins de 4000 ticks) : deplacement en lignes droites avec controle de distance
 *   - Ennemi trop proche, on recule
 *   - Rien , on avance tout droit
 *   - Bloque devant, on recule et on laisse la Base gerer le deblocage
 *
 * Phase 2 (4000 ticks et plus), on change de méthode de deplacement
 *   - Trop loin de l'ennemi, il se rapproche
 *   - Trop pres de l'ennemi, il s'eloigne vers le point oppose
 *   - Dans la zone morte, il orbite lateralement autour de l'ennemi
 *
 * Priorites :
 *   - Deblocage mur : runModeDeblocage() est verifie avant toute logique de deplacement
 *   - Fuite en cas de PV trop bas : fuir vers le bord allie si la sante tombe sous 20%
 */

public class TeamASecondaryBotChenFallavierTang extends PatientHunterBase {

    private static final int ETAT_AVANCE = 1;
    private static final int ETAT_DEMI_TOUR = 2;

    // Apres un nombre de ticks, on change de strategie de deplacement (seDeplacerVers)
    private static final int TICK_DEBUT_PHASE2 = 4000;

    // Distance ideale a maintenir avec l'ennemi
    private static final double DISTANCE_SOUHAITEE = 600.0;
    // si l'ecart a la distance souhaitee est inferieur a cette valeur, on ne corrige pas
    private static final double MARGE_DISTANCE = 120.0;

    // En dessous de 20% des PV , le robot fuit
    private static final double RATIO_PV_FUITE = 0.20;

    private double santeInitiale;// PV au debut de la partie, sert de reference pour la fuite
    private int etatActuel;// etat courant de la phase 1
    private int ticksDemiTourRestants; // nombre de stepTurn restants pour finir le demi-tour

    @Override
    public void activate() {
        // Initialisation commune (position, vitesse, tables de tracking)
        activerBase();
        // Memoriser la sante de depart pour detecter les PV critiques plus tard
        santeInitiale = getHealth();
        // Demarrer en mode avance
        etatActuel = ETAT_AVANCE;
        ticksDemiTourRestants = 0;
    }

    @Override
    public void step() {
        if (getHealth() <= 0) return;
        // Etape commune : mise a jour odometrie, radar, messages broadcast
        ArrayList<PointSuivi> ennemisRadarCeStep = stepBase();
        // Afficher l'etat actuel dans l'interface
        publierStatut(compteurStep >= TICK_DEBUT_PHASE2 ? "NAV" : (etatActuel == ETAT_DEMI_TOUR ? "DEMITOUR" : "AVA"));
        // Priorite 1 : fuite si les PV sont trop bas
        if (doitFuir()) {
            fuirAvecPeuDeVie(ennemisRadarCeStep);
            return;
        }
        // Priorite 2 : si un deblocage mur est en cours, on le finit completement avant tout
        if (executerDeblocageDroite()) return;
        // Priorite 3 : choisir la phase de comportement selon le nombre de ticks ecoules
        if (compteurStep >= TICK_DEBUT_PHASE2) executerPhase2(ennemisRadarCeStep);
        else executerPhase1(ennemisRadarCeStep);
    }

    // Phase 1 : lignes droites avec demi-tour si on bloque

    // Gere l'etat courant de la phase 1 : demi-tour ou avance selon la situation
    private void executerPhase1(ArrayList<PointSuivi> ennemisRadarCeStep) {
        // Si un demi-tour est en cours, on continue de tourner jusqu'a ce qu'il soit termine
        if (etatActuel == ETAT_DEMI_TOUR) {
            stepTurn(Parameters.Direction.LEFT);
            tentativeAvanceStep = false;
            // Decremente le compteur, et repasse en mode avance une fois le demi-tour termine
            if (--ticksDemiTourRestants <= 0) etatActuel = ETAT_AVANCE;
            return;
        }
        // Sinon on applique la stratégie d'avancer et de reculer
        executerAvanceOuRecul(ennemisRadarCeStep);
        }
        // Avance tout droit si la voie est libre et l'ennemi assez loin sinon on recule
        private void executerAvanceOuRecul(ArrayList<PointSuivi> ennemisRadarCeStep) {
        double distanceEnnemiLePlusProche = trouverDistanceEnnemiLePlusProche(ennemisRadarCeStep);
        // Si l'ennemi est trop proche, on recule
        if (distanceEnnemiLePlusProche < DISTANCE_SOUHAITEE - MARGE_DISTANCE) {
            reculerAvecOdometrie();
            tentativeAvanceStep = false;
            return;
        }
        // Si la voie est libre devant, avancer
        IFrontSensorResult.Types typeFront = detectFront().getObjectType();
        if (typeFront == IFrontSensorResult.Types.NOTHING || typeFront == IFrontSensorResult.Types.BULLET) {
        avancerAvecOdometrie();
        return;
        }

        // Si devant est bloquer alors on recule et on laisse le stuck detector de la Base declencher le deblocage
        reculerAvecOdometrie();
        tentativeAvanceStep = false;
    }

    // Phase 2 : navigation active avec seDeplacerVers()

    // Controle la distance a l'ennemi et orbite autour de lui en utilisant seDeplacerVers()
    private void executerPhase2(ArrayList<PointSuivi> ennemisRadarCeStep) {
        PointSuivi ennemiLePlusProche = trouverEnnemiLePlusProche(ennemisRadarCeStep);
        // Aucun ennemi detecter alors on avance tout droit en attendant d'en trouver un
        if (ennemiLePlusProche == null) {
            avancerAvecOdometrie();
            return;
        }
        double dx = ennemiLePlusProche.x - monX;
        double dy = ennemiLePlusProche.y - monY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > DISTANCE_SOUHAITEE + MARGE_DISTANCE) {
            // Trop loin : se rapprocher de l'ennemi
            seDeplacerVers(ennemiLePlusProche.x, ennemiLePlusProche.y);
        } else if (distance < DISTANCE_SOUHAITEE - MARGE_DISTANCE) {
            // Trop pres : fuir vers le point  oppose a l'ennemi
            double pointFuiteX = borner(monX - dx, CARTE_MIN_X + 50, CARTE_MAX_X - 50);
            double pointFuiteY = borner(monY - dy, CARTE_MIN_Y + 50, CARTE_MAX_Y - 50);
            seDeplacerVers(pointFuiteX, pointFuiteY);
        } else {
            // on orbite autour de l'ennemi
            // Le sens d'orbite alterne selon l'idInstance pour que les deux secondary n'orbitent pas pareil
            double sensOrbite = (idInstance % 2 == 0) ? 1.0 : -1.0;
            double directionPerp = Math.atan2(dy, dx) + sensOrbite * (Math.PI / 2.0);
            double cibleOrbitX = borner(monX + 300.0 * Math.cos(directionPerp), CARTE_MIN_X + 50, CARTE_MAX_X - 50);
            double cibleOrbitY = borner(monY + 300.0 * Math.sin(directionPerp), CARTE_MIN_Y + 50, CARTE_MAX_Y - 50);
            seDeplacerVers(cibleOrbitX, cibleOrbitY);
        }
    }

    //Detection et deblocage mur droite

    // Retourne la distance au plus proche ennemi connu (radar direct prioritaire)
    private double trouverDistanceEnnemiLePlusProche(ArrayList<PointSuivi> ennemisRadarCeStep) {
    PointSuivi p = trouverEnnemiLePlusProche(ennemisRadarCeStep);
    if (p == null) return Double.MAX_VALUE;
        double dx = p.x - monX, dy = p.y - monY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // Retourne le point ennemi le plus proche : radar direct d'abord, puis table broadcast
    private PointSuivi trouverEnnemiLePlusProche(ArrayList<PointSuivi> ennemisRadarCeStep) {
        PointSuivi meilleur = null;
        double meilleureDistance2 = Double.MAX_VALUE;
        // Parcourir les ennemis detectes directement par le radar ce step
        for (PointSuivi p : ennemisRadarCeStep) {
            double dx = p.x - monX, dy = p.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { 
                meilleureDistance2 = d2;
                meilleur = p;
            }
        }
        // Si le radar a trouve quelque chose, on s'arrete la
        if (meilleur != null) return meilleur;

        // Sinon on cherche dans la table partagee (ennemis broadcast par les allies)
        for (PointSuivi p : listeEnnemis) {
            // Ignorer les entrees trop vieilles
            if (compteurStep - p.dernierStepMiseAJour > TTL_ENNEMI_RADAR) continue;
            double dx = p.x - monX, dy = p.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { 
                meilleureDistance2 = d2;
                meilleur = p;
            }
        }
        return meilleur;
    }

    // Fuite si les PV sont trop bas 

    // Retourne vrai si la sante est tombee sous le seuil de fuite
    private boolean doitFuir() {
        return getHealth() <= Math.max(1.0, santeInitiale * RATIO_PV_FUITE);
    }

    // Fuit en direction opposee a l'ennemi le plus proche ou vers le bord allie si il y a aucun enemi
    private void fuirAvecPeuDeVie(ArrayList<PointSuivi> ennemisRadarCeStep) {
        PointSuivi menace = trouverEnnemiLePlusProche(ennemisRadarCeStep);
        // Calculer la direction de fuite : oppose a l'ennemi ou vers notre bord si aucun ennemi visible
        double directionFuite = (menace != null) ? Math.atan2(monY - menace.y, monX - menace.x): (estEquipeA ? Parameters.WEST : Parameters.EAST);
        double delta = normaliserAngle(directionFuite - getHeading());
        double seuilAlignement = monAngleTourParStep * MULTIPLICATEUR_SEUIL_ALIGNEMENT;
        // Si pas encore aligne vers la direction de fuite : on tourne d'abord
        if (Math.abs(delta) > seuilAlignement) {
            // delta > 0 = cible a gauche , donc on tourne a gauche
            stepTurn(delta > 0.0 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
            tentativeAvanceStep = false;
            return;
        }
        // Aligne : on avance si possible sinon on recule et on esquive
        IFrontSensorResult.Types typeFront = detectFront().getObjectType();
        if (typeFront == IFrontSensorResult.Types.NOTHING || typeFront == IFrontSensorResult.Types.BULLET) {
            avancerAvecOdometrie();
        } else {
            reculerAvecOdometrie();
            stepTurn(Math.random() < 0.5 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
            tentativeAvanceStep = false;
        }
    }
}