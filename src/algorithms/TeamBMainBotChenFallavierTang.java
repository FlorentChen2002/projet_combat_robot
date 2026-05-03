package algorithms;

import java.util.ArrayList;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
/**
 * PatientHunterMain / TeamBMainBotChenFallavierTang: le robot principal
 * - Au debut de la partien il se separe
 * - Il cherche d'abord une cible propre, tente un tir si c'est raisonnable,
 * - puis avance ou se debloque si rien de mieux n'est possible
 */
public class TeamBMainBotChenFallavierTang extends PatientHunterBase {

    private static final double CLOSE_WRECK_DISTANCE = 450.0;
    private static final int FIRE_TARGET_MAX_AGE_STEPS = 6;
    private int ticksDroiteDepartRestants;
    private double capSeparationDepart;

    @Override
    public void activate() {
        activerBase();
        // Le plan de depart decide dans quelle direction partir au debut
        PlanDepart plan = construirePlanDepart();
        capSeparationDepart = plan.capSeparation;
        ticksDroiteDepartRestants = plan.ticksDroite;
    }

    @Override
    public void step() {
        if (getHealth() <= 0) return;
        // Mise a jour commune: position, messages, radar, memoire partagee
        ArrayList<PointSuivi> ennemisRadarCeStep = stepBase();
        if (ennemisRadarCeStep == null) return;
        // Tant que la phase de depart n'est pas finie, elle garde la main
        if (executerStrategieDepart()) {
            publierStatut("STARTUP");
            return;
        }
        // Petit label pour savoir rapidement ce que fait le robot
        String state = modeDeblocageDroite ? "UNBLOCK" : aEnnemVerrouille && compteurStep - dernierStepVerrouEnnemi <= STEPS_MAINTIEN_VERROU ? "LOCK" : "SEEK";
        publierStatut(state);
        // On choisit la cible la plus interessante, puis on la garde un peu
        // pour eviter que le robot change d'avis toutes les secondes
        CandidatCible cibleBrute = selectionnerCibleParPriorite();
        CandidatCible target = appliquerVerrouCible(cibleBrute);
        // Si la cible est encore vife
        PointSuivi pointDeTir = selectionnerPointTirFraisDepuisCible(cibleBrute);
        ArrayList<IRadarResult> radar = detectRadar();
        if (tentativeTir(pointDeTir, radar)) return;
        // Sinon on se deplace vers la cible retenue
        if (target != null) {
            seDeplacerVers(target.point.x, target.point.y);
            return;
        }
        // Pas de cible, on tente juste de se deplacer ou on enclenche le deblocage si un obstacle bloque le passage
        if (executerDeblocageDroite()) return;
        IFrontSensorResult.Types idleFront = detectFront().getObjectType();
        if (idleFront == IFrontSensorResult.Types.NOTHING) {
            stepsBlockeAvant = 0;
            avancerAvecOdometrie();
        } else {
            stepsBlockeAvant++;
            if (!modeDeblocageDroite || stepsDeblocageDroiteRestants <= 0) {
                modeDeblocageDroite = true;
                stepsDeblocageDroiteRestants = MAX_STEPS_DEBLOCAGE_DROITE;
            }
            reculerAvecOdometrie();
            stepTurn(Parameters.Direction.RIGHT);
        }
    }
    // Au debut de la partie, on ecarte les tireurs pour eviter qu'ils se
    // marchent dessus: un part vers le haut, l'autre vers le bas, puis ils
    // reviennent progressivement sur l'axe principal
    private boolean executerStrategieDepart() {
        if (ticksDroiteDepartRestants > 0) {
            if (capSeparationDepart == Parameters.EAST) {
                if (idInstance == 1){
                    if(ticksDroiteDepartRestants > 355) {
                        stepTurn(Parameters.Direction.LEFT);
                    }
                    else if(ticksDroiteDepartRestants<=25) {
                        stepTurn(Parameters.Direction.RIGHT);
                    }
                    else avancerAvecOdometrie();
                }
                if (idInstance == 3 ) {
                    if (ticksDroiteDepartRestants > 355) {
                        stepTurn(Parameters.Direction.RIGHT);
                    }
                    else if(ticksDroiteDepartRestants<=25) {
                        stepTurn(Parameters.Direction.LEFT);
                    }
                    else avancerAvecOdometrie();
                }
                ticksDroiteDepartRestants--;
                return true;
            } else {
                // Meme logique, mais retournee quand l'equipe part de l'Ouest.
                if (idInstance == 1) {
                    if (ticksDroiteDepartRestants > 355) stepTurn(Parameters.Direction.RIGHT);
                    else if (ticksDroiteDepartRestants <= 25) stepTurn(Parameters.Direction.LEFT);
                    else avancerAvecOdometrie();
                }
                if (idInstance == 3) {
                    if (ticksDroiteDepartRestants > 355) stepTurn(Parameters.Direction.LEFT);
                    else if (ticksDroiteDepartRestants <= 25) stepTurn(Parameters.Direction.RIGHT);
                    else avancerAvecOdometrie();
                }
                ticksDroiteDepartRestants--;
                return true;
            }
        }
        return false;
    }

    // On ne tire que si la cible est encore d'actualité pour eviter de tirer sur une position obsolete
    private PointSuivi selectionnerPointTirFraisDepuisCible(CandidatCible cibleBrute) {
        if (cibleBrute == null) return null;
        if (compteurStep - cibleBrute.point.dernierStepMiseAJour > FIRE_TARGET_MAX_AGE_STEPS) return null;
        return cibleBrute.point;
    }
    public double calculerAngleStable(double dx, double dy) {
        final double a;
        if (dy == 0.) {
            a = (dx > 0) ? 0 : Math.PI;
        } else if (dx == 0.) {
            a = (dy > 0) ? Math.PI / 2 : -Math.PI / 2;
        } else {
            a = Math.atan2(dy, dx); 
        }
    return normaliserAngle(a);
    }
    // Tir direct: on verifie la portee et s'il y a un allie dans l'axe ou un obstacle trop proche devant
    private boolean tentativeTir(PointSuivi pointDeTir, ArrayList<IRadarResult> radar) {
        if (pointDeTir == null) return false;
        double dx = pointDeTir.x - monX, dy = pointDeTir.y - monY;
        double d2 = dx * dx + dy * dy;
        double distanceTir = Math.sqrt(d2);
        if (distanceTir > Parameters.bulletRange) return false;
        double directionTir = calculerAngleStable(dx, dy);
        boolean allieDevant = allieDevant();
        boolean allieLigneDepuisListe = allieDansLigneDeViseeDepuisListe(directionTir, distanceTir);
        boolean allieLigneDepuisRadar = allieDansLigneDeViseeDepuisRadar(radar, directionTir, distanceTir);
        IFrontSensorResult.Types objetDevantCanon = detectFront().getObjectType();
        boolean epaveProchDevant = objetDevantCanon == IFrontSensorResult.Types.Wreck && distanceTir <= CLOSE_WRECK_DISTANCE;
        if (!allieDevant && !allieLigneDepuisListe && !allieLigneDepuisRadar && !epaveProchDevant) {
            tirerAvecIndice(directionTir);
            return true;
        }
        if (epaveProchDevant) {
            reculerAvecOdometrie();
            stepTurn(Parameters.Direction.RIGHT);
            return true;
        }
        return false;
    }
}