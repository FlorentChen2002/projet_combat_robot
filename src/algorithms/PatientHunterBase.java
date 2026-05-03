/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import java.util.ArrayList;

import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;

public abstract class PatientHunterBase extends Brain {

    // Cette classe regroupe tout ce que les deux robots ont en commun:
    // memoire des ennemis, suivi des balles, communication d'equipe,
    // deplacement, gestion des blocages et quelques regles de tir ami

    // Constantes communes aux deux robots
    protected static final double DISTANCE_FUSION_ROBOT = 220.0;
    protected static final double DISTANCE_ASSOCIATION_PISTE_BALLE = 180.0;
    protected static final int AGE_MAX_PISTE_BALLE = 8;
    protected static final double DISTANCE_CORRESPONDANCE_TIREUR = 280.0;
    protected static final double CONE_DIRECTION_PROJECTILE_ALLIE = Math.PI / 6.0;
    protected static final double ECART_MAX_SOURCE_PROJECTILE_ALLIE = 900.0;
    protected static final double DISTANCE_TIREUR_INFERE = 500.0;
    protected static final int INFERRER_SEULEMENT_SI_AUCUN_ENNEMI_DEPUIS = 3;
    protected static final int MAX_PISTES_ENNEMIES = 5;
    protected static final double CARTE_MIN_X = 0.0, CARTE_MAX_X = 3000.0, CARTE_MIN_Y = 0.0, CARTE_MAX_Y = 2000.0;
    protected static final String PREFIXE_MSG = "PHR";
    protected static final String TYPE_MSG_ENNEMI = "ENEMY";
    protected static final String TYPE_MSG_ENNEMI_PREDIT = "ENEMY_PREDICTED";
    protected static final String TYPE_MSG_BALLE = "BULLET";
    protected static final String TYPE_MSG_TIR = "FIRE";
    protected static final String TYPE_MSG_POS = "POS";
    protected static final int TTL_DIFFUSION = 30;
    protected static final int TTL_ENNEMI_RADAR = 25;
    protected static final int TTL_BALLE_RADAR = 10;
    protected static final int TTL_INDICE_TIR = 18;
    protected static final double DISTANCE_MEMOIRE_ENNEMI_LOIN = 1000.0;
    protected static final double MULTIPLICATEUR_SEUIL_ALIGNEMENT  = 4.0;
    protected static final double ANGLE_CONE_AVANT = 0.26 * Math.PI;
    protected static final double ANGLE_SORTIE_REORIENTATION = 0.10 * Math.PI;
    protected static final double BANDE_CENTRALE_DEPART = 220.0;
    protected static final double ANGLE_FLANC_DEPART = 0.20 * Math.PI;
    protected static final double DISTANCE_MIN_BLOCAGE = 0.5;
    protected static final double RATIO_MOUVEMENT_BLOCAGE = 0.15;
    protected static final int SEUIL_BLOCAGE_AVANT = 3;
    protected static final int STEPS_ROTATION_DEBLOCAGE = 3;
    protected static final int STEPS_DROITE_APRES_SEPARATION = 380;
    protected static final int MAX_STEPS_DEBLOCAGE_DROITE = 100;
    protected static final int PERIODE_ZIGZAG_DEBLOCAGE = 4;
    protected static final int STEPS_MAINTIEN_VERROU = 25;

    protected static final int NOMBRE_ROBOTS_EQUIPE = 5;
    protected static final int TTL_POSITION_ALLIE = 40;
    protected static final double ANGLE_BLOCAGE_TIR_FRATRICIDE = Math.PI / 20.0;
    protected static final double EXTENSION_ARRIERE_TIR_FRATRICIDE = 450.0;
    protected static final double MARGE_EXTRA_HITBOX_FRATRICIDE = 70.0;
    protected static final double MARGE_EXTRA_RADAR_FRATRICIDE  = 40.0;
    protected static final double RAYON_MAX_HITBOX_ALLIE = Math.max( Math.max(Parameters.teamAMainBotRadius, Parameters.teamASecondaryBotRadius), Math.max(Parameters.teamBMainBotRadius, Parameters.teamBSecondaryBotRadius));

    // Tableau de suivi des robots / projectilles enemis / Allier
    protected final ArrayList<PointSuivi> listeEnnemis = new ArrayList<>();
    protected final ArrayList<PointSuivi> listeProjectiles = new ArrayList<>();
    protected final ArrayList<PisteBalle> pistesBalles = new ArrayList<>();
    protected final ArrayList<IndiceTirRecent> indicesTirRecents = new ArrayList<>();
    protected final ArrayList<PositionAllie> positionsAllies = new ArrayList<>();
    protected final ArrayList<PointSuivi> ennemisRadarCeStep = new ArrayList<>();

    // Variables en communs avec les robots main et secondary
    protected boolean estEquipeA, estTireur;
    protected double monX, monY, maVitesse, monAngleTourParStep;
    protected int idInstance, compteurStep, dernierStepEnnemDirect, delaiDiffusionPosition;
    protected boolean aEnnemVerrouille, aVerrouCible;
    protected double xEnnemVerrouille, yEnnemVerrouille, xVerrouCible, yVerrouCible;
    protected int dernierStepVerrouEnnemi, prioriteVerrouCible, verrouilleJusquauStep;
    protected int dernierStepTir;
    protected double derniereDirTir;
    protected int stepsBlockeAvant, stepsRotationForcee, stepsDeblocageDroiteRestants;
    protected Parameters.Direction dirRotationForcee;
    protected boolean modeReorientation, aPositionMesuree, tentativeAvanceStep, modeDeblocageDroite;
    protected double xDerniereMesure, yDerniereMesure;

    // static d'initialisation d'id 
    protected static int PROCHAIN_ID_INSTANCE = 1;
    protected static int IndexBotPrincipalA = 0, IndexBotPrincipalB = 0;
    protected static int IndexBotSecondaireA = 0, IndexBotSecondaireB = 0;
    protected enum TypeSource { RADAR, DIFFUSION }
    protected enum AliasProjectile { INCONNU, ALLIE, ENNEMI }

    // Remet les variables du robots à zéro
    protected void activerBase() {
        if (idInstance == 0) idInstance = acquerirIdInstance();
        estTireur = getHealth() > 150.0;
        estEquipeA   = Math.cos(getHeading() - Parameters.EAST) > 0.0;
        if (IndexBotPrincipalA >= 3 || IndexBotSecondaireA >= 2
                || IndexBotPrincipalB >= 3 || IndexBotSecondaireB >= 2) {
            IndexBotPrincipalA = IndexBotSecondaireA = IndexBotPrincipalB = IndexBotSecondaireB = 0;
        }
        assignerPositionDepart();
        listeEnnemis.clear();
        listeProjectiles.clear();
        pistesBalles.clear();
        indicesTirRecents.clear();
        positionsAllies.clear();
        for (int i = 0; i < NOMBRE_ROBOTS_EQUIPE; i++) positionsAllies.add(new PositionAllie());
        monX = borner(monX, CARTE_MIN_X, CARTE_MAX_X);
        monY = borner(monY, CARTE_MIN_Y, CARTE_MAX_Y);
        aPositionMesuree = true;
        xDerniereMesure = monX;
        yDerniereMesure = monY;
        tentativeAvanceStep = false;
        modeDeblocageDroite = false;
        stepsDeblocageDroiteRestants = 0;
        compteurStep = 0;
        delaiDiffusionPosition = 0;
        dernierStepEnnemDirect = -10000;
        aEnnemVerrouille = false;
        dernierStepVerrouEnnemi = -10000;
        aVerrouCible = false;
        verrouilleJusquauStep = -1;
        dernierStepTir = -10000;
        derniereDirTir = Double.NaN;
        stepsBlockeAvant = 0;
        stepsRotationForcee = 0;
        dirRotationForcee = Parameters.Direction.LEFT;
        modeReorientation = false;
        mettreAJourPositionAllie(idInstance, monX, monY);
    }

    // Chosi la position en fonction de son equipe (A ou B) et le role (tireur ou secondaire)
    private void assignerPositionDepart() {
        if (estEquipeA) {
            if (estTireur) {
                IndexBotPrincipalA++;
                if (IndexBotPrincipalA == 1) { 
                    monX = Parameters.teamAMainBot1InitX;
                    monY = Parameters.teamAMainBot1InitY;
                } else if (IndexBotPrincipalA == 2) {
                    monX = Parameters.teamAMainBot2InitX;
                    monY = Parameters.teamAMainBot2InitY;
                }else {
                    monX = Parameters.teamAMainBot3InitX;
                    monY = Parameters.teamAMainBot3InitY;
                }
                maVitesse = Parameters.teamAMainBotSpeed;
                monAngleTourParStep = Parameters.teamAMainBotStepTurnAngle;
            } else {
                IndexBotSecondaireA++;
                if (IndexBotSecondaireA == 1) {
                    monX = Parameters.teamASecondaryBot1InitX;
                    monY = Parameters.teamASecondaryBot1InitY;
                } else {
                    monX = Parameters.teamASecondaryBot2InitX;
                    monY = Parameters.teamASecondaryBot2InitY;
                }
                maVitesse = Parameters.teamASecondaryBotSpeed;
                monAngleTourParStep = Parameters.teamASecondaryBotStepTurnAngle;
            }
        } else {
            if (estTireur) {
                IndexBotPrincipalB++;
                if (IndexBotPrincipalB == 1) {
                    monX = Parameters.teamBMainBot1InitX;
                    monY = Parameters.teamBMainBot1InitY;
                } else if (IndexBotPrincipalB == 2) {
                    monX = Parameters.teamBMainBot2InitX;
                    monY = Parameters.teamBMainBot2InitY;
                } else {
                    monX = Parameters.teamBMainBot3InitX;
                    monY = Parameters.teamBMainBot3InitY;
                }
                maVitesse = Parameters.teamBMainBotSpeed;
                monAngleTourParStep = Parameters.teamBMainBotStepTurnAngle;
            } else {
                IndexBotSecondaireB++;
                if (IndexBotSecondaireB == 1) {
                    monX = Parameters.teamBSecondaryBot1InitX;
                    monY = Parameters.teamBSecondaryBot1InitY;
                } else {
                    monX = Parameters.teamBSecondaryBot2InitX;
                    monY = Parameters.teamBSecondaryBot2InitY;
                }
                maVitesse = Parameters.teamBSecondaryBotSpeed;
                monAngleTourParStep = Parameters.teamBSecondaryBotStepTurnAngle;
            }
        }
    }

    // On recale d'abord la position, on traite les messages de l'equipe,
    // puis on lit le radar et on met a jour les ennemis/projeciles vus
    protected ArrayList<PointSuivi> stepBase() {
        verifierBlocageEtMettreAJourDeblocage();
        tentativeAvanceStep = false;
        if (aPositionMesuree) {
            xDerniereMesure = monX;
            yDerniereMesure = monY;
        }
        compteurStep++;
        mettreAJourPositionAllie(idInstance, monX, monY);
        traiterMessagesEntrants();
        if (delaiDiffusionPosition <= 0) {
            diffuserDetection(TYPE_MSG_POS, monX, monY, Double.NaN);
            delaiDiffusionPosition = 2;
        } else {
            delaiDiffusionPosition--;
        }
        nettoyerPistesBallesExpirees();
        nettoyerPointsDiffusionPerimes();
        ArrayList<IRadarResult> radar = detectRadar();
        ArrayList<IRadarResult> radarEnnemis = new ArrayList<>(radar);
        radarEnnemis.removeIf(rr -> rr.getObjectType() != IRadarResult.Types.OpponentMainBot && rr.getObjectType() != IRadarResult.Types.OpponentSecondaryBot);
        ennemisRadarCeStep.clear();
        radarEnnemis.forEach(rr -> {
            double x = monX + Math.cos(rr.getObjectDirection()) * rr.getObjectDistance();
            double y = monY + Math.sin(rr.getObjectDirection()) * rr.getObjectDistance();
            dernierStepEnnemDirect = compteurStep;
            mettreAJourEnnemI(x, y, true, TypeSource.RADAR, false);
            ennemisRadarCeStep.add(new PointSuivi(x, y, TypeSource.RADAR, compteurStep, false));
            diffuserDetection(TYPE_MSG_ENNEMI, x, y, Double.NaN);
        });
        // prediction des tirs ennemis a partir du radar de balles
        for (IRadarResult r : radar) {
            if (r.getObjectType() != IRadarResult.Types.BULLET) continue;
            double x = monX + r.getObjectDistance() * Math.cos(r.getObjectDirection());
            double y = monY + r.getObjectDistance() * Math.sin(r.getObjectDirection());
            if (estProbablementMaPropeBalle(x, y)) continue;
            if (estProbablementTirDansDirectionAllie(x, y)) continue;
            if (estProbablementBalleAllie(x, y)) continue;
            PredictionSource prediction = estimerDirectionTirPourPiste(x, y);
            if (prediction != null) { // si on a pu faire une prediction fiable de la source du tir, on met a jour la memoire des ennemis avec cette position de tireur et on diffuse cette info a l'equipe 
                if (prediction.alias == AliasProjectile.ALLIE) continue;
                mettreAJourProjectile(prediction.x, prediction.y, TypeSource.RADAR);
                diffuserDetection(TYPE_MSG_ENNEMI_PREDIT, prediction.x, prediction.y, prediction.directionSource);
            } else { // si on ne peut pas faire de prediction fiable, on fait quand meme une mise a jour de la memoire des ennemis pour eviter de se faire surprendre par une balle qu'on aurait pas vu arriver
                mettreAJourProjectile(x, y, TypeSource.RADAR);
                double directionSource = Math.atan2(monY - y, monX - x);
                double ex = borner(monX + DISTANCE_TIREUR_INFERE * Math.cos(directionSource), CARTE_MIN_X, CARTE_MAX_X);
                double ey = borner(monY + DISTANCE_TIREUR_INFERE * Math.sin(directionSource), CARTE_MIN_Y, CARTE_MAX_Y);
                AliasProjectile alias = classerTireurInfere(ex, ey);
                if (alias == AliasProjectile.ALLIE) continue;
                if (compteurStep - dernierStepEnnemDirect > INFERRER_SEULEMENT_SI_AUCUN_ENNEMI_DEPUIS) {
                    mettreAJourEnnemI(ex, ey, false, TypeSource.RADAR, true);
                }
                diffuserDetection(TYPE_MSG_ENNEMI_PREDIT, ex, ey, directionSource);
            }
        }
        mettreAJourVerrouEnnemDepuisRadar(ennemisRadarCeStep);
        synchroniserListeProjectilesDepuisPistes();
        return ennemisRadarCeStep;
    }

    // verification si il est de droit de partir a droite ou a gauche et assignation du cap de separation
    protected PlanDepart construirePlanDepart() {
        double capSeparation = estEquipeA ? Parameters.EAST : Parameters.WEST;
        if (!estTireur) return new PlanDepart(capSeparation, 0);
        return new PlanDepart(capSeparation, STEPS_DROITE_APRES_SEPARATION);
    }

    // Detecte si le robot est bloque et active le mode de deblocage
    private void verifierBlocageEtMettreAJourDeblocage() {
        if (tentativeAvanceStep && aPositionMesuree) {
            double dxm = monX - xDerniereMesure;
            double dym = monY - yDerniereMesure;
            double deplacement2 = dxm * dxm + dym * dym;
            double epsilonBlocage = Math.max(DISTANCE_MIN_BLOCAGE, maVitesse * RATIO_MOUVEMENT_BLOCAGE);
            if (deplacement2 <= epsilonBlocage * epsilonBlocage) {
                modeDeblocageDroite = true;
                if (stepsDeblocageDroiteRestants <= 0) stepsDeblocageDroiteRestants = MAX_STEPS_DEBLOCAGE_DROITE;
                stepsRotationForcee = Math.max(stepsRotationForcee, STEPS_ROTATION_DEBLOCAGE);
                dirRotationForcee = Parameters.Direction.RIGHT;
            }
        }
    }

    // Tracking des projectilles

    // Quand on voit une balle bouger, on essaye de deviner d'ou elle est partie
    // Si la piste semble coherente, on en deduit un tireur probable et on met a jour la memoire des ennemis avec cette position de tireur
    protected PredictionSource estimerDirectionTirPourPiste(double x, double y) {
        PisteBalle piste = assignerAPiste(x, y);
        if (piste == null || !piste.aPrecedent) return null;
        double vx = piste.lastX - piste.prevX;
        double vy = piste.lastY - piste.prevY;
        double norm = Math.sqrt(vx * vx + vy * vy);
        if (norm < 1e-6) return null;
        double directionSource = Math.atan2(-vy, -vx);
        double ex = borner(monX + DISTANCE_TIREUR_INFERE * Math.cos(directionSource), CARTE_MIN_X, CARTE_MAX_X);
        double ey = borner(monY + DISTANCE_TIREUR_INFERE * Math.sin(directionSource), CARTE_MIN_Y, CARTE_MAX_Y);
        AliasProjectile alias = classerTireurInfere(ex, ey);
        piste.alias = alias;
        if (alias != AliasProjectile.ALLIE && compteurStep - dernierStepEnnemDirect > INFERRER_SEULEMENT_SI_AUCUN_ENNEMI_DEPUIS) {
            mettreAJourEnnemI(ex, ey, false, TypeSource.RADAR, true);
        }
        return new PredictionSource(ex, ey, directionSource, alias);
    }

    // Verifie si le tireur probable ressemble plutot a un allie ou a un ennemi
    protected AliasProjectile classerTireurInfere(double sx, double sy) {
        double distSoi2 = (sx - monX) * (sx - monX) + (sy - monY) * (sy - monY);
        double distCorrespondance2 = DISTANCE_CORRESPONDANCE_TIREUR * DISTANCE_CORRESPONDANCE_TIREUR;
        if (distSoi2 <= distCorrespondance2) return AliasProjectile.ALLIE;
        for (int i = 0; i < positionsAllies.size(); i++) {
            int idEmetteur = i + 1;
            if (idEmetteur == idInstance) continue;
            PositionAllie ap = positionsAllies.get(i);
            if (!ap.known || compteurStep - ap.dernierStepMiseAJour > TTL_POSITION_ALLIE) continue;
            double d2 = (sx - ap.x) * (sx - ap.x) + (sy - ap.y) * (sy - ap.y);
            if (d2 <= distCorrespondance2) return AliasProjectile.ALLIE;
        }
        if (aEnnemVerrouille && compteurStep - dernierStepVerrouEnnemi <= STEPS_MAINTIEN_VERROU) {
            double d2 = (sx - xEnnemVerrouille) * (sx - xEnnemVerrouille) + (sy - yEnnemVerrouille) * (sy - yEnnemVerrouille);
            if (d2 <= distCorrespondance2) return AliasProjectile.ENNEMI;
        }
        for (PointSuivi p : listeEnnemis) {
            double d2 = (sx - p.x) * (sx - p.x) + (sy - p.y) * (sy - p.y);
            if (d2 <= distCorrespondance2) return AliasProjectile.ENNEMI;
        }
        return AliasProjectile.INCONNU;
    }

    // Si on vient de tirer et que la balle detectee, on fait une vérification si le tir provient d'un allié ou un enemi
    protected boolean estProbablementMaPropeBalle(double x, double y) {
        if (compteurStep - dernierStepTir > 3 || Double.isNaN(derniereDirTir)) return false;
        double dx = x - monX, dy = y - monY;
        double dist2 = dx * dx + dy * dy;
        if (dist2 > 700.0 * 700.0) return false;
        double bulletDirection = Math.atan2(dy, dx);
        return Math.abs(normaliserAngle(bulletDirection - derniereDirTir)) <= Math.PI / 6.0;
    }

    // Essaie d'ignorer les balles si elle provient d'un tir d'un allie
    // et qui ne devraient pas etre comptees comme un enemi
    protected boolean estProbablementTirDansDirectionAllie(double x, double y) {
        double bulletToMeX = monX - x, bulletToMeY = monY - y;
        double bulletDist = Math.sqrt(bulletToMeX * bulletToMeX + bulletToMeY * bulletToMeY);
        if (bulletDist < 1.0) return false;
        double bulletDirToMe = Math.atan2(bulletToMeY, bulletToMeX);
        for (int i = 0; i < positionsAllies.size(); i++) {
            int idEmetteur = i + 1;
            if (idEmetteur == idInstance) continue;
            PositionAllie ap = positionsAllies.get(i);
            if (!ap.known || compteurStep - ap.dernierStepMiseAJour > TTL_POSITION_ALLIE) continue;
            double allyToMeX = monX - ap.x, allyToMeY = monY - ap.y;
            double distanceAllie = Math.sqrt(allyToMeX * allyToMeX + allyToMeY * allyToMeY);
            if (distanceAllie <= bulletDist) continue;
            if (distanceAllie - bulletDist > ECART_MAX_SOURCE_PROJECTILE_ALLIE) continue;
            double allyDirToMe = Math.atan2(allyToMeY, allyToMeX);
            if (Math.abs(normaliserAngle(allyDirToMe - bulletDirToMe)) <= CONE_DIRECTION_PROJECTILE_ALLIE)
                return true;
        }
        return false;
    }

    // il reconnait une balle qui correspond a un message "je viens de tirer" venu d'un allie
    protected boolean estProbablementBalleAllie(double x, double y) {
        for (IndiceTirRecent hint : indicesTirRecents) {
            if (hint.idEmetteur == idInstance) continue;
            int age = compteurStep - hint.dernierStepMiseAJour;
            if (age < 0 || age > TTL_INDICE_TIR) continue;
            double expectedX = hint.x + age * Parameters.bulletVelocity * Math.cos(hint.direction);
            double expectedY = hint.y + age * Parameters.bulletVelocity * Math.sin(hint.direction);
            double dx = x - expectedX;
            double dy = y - expectedY;
            if (dx * dx + dy * dy <= 110.0 * 110.0) return true;
        }
        return false;
    }

    // Memorise le dernier tir annonce par un allie, ou met a jour l'ancien si on l'a deja
    protected void ajouterIndiceTirRecent(int idEmetteur, double x, double y, double direction) {
        for (IndiceTirRecent hint : indicesTirRecents) {
            if (hint.idEmetteur == idEmetteur) {
                hint.x = x;
                hint.y = y;
                hint.direction = direction;
                hint.dernierStepMiseAJour = compteurStep;
                return;
            }
        }
        indicesTirRecents.add(new IndiceTirRecent(idEmetteur, x, y, direction, compteurStep));
    }

    // Lie une nouvelle observation de balle a une piste deja connue
    // ou cree une piste neuve si rien ne correspond
    private PisteBalle assignerAPiste(double x, double y) {
        PisteBalle meilleur       = null;
        double      meilleureDistance2  = Double.MAX_VALUE;
        for (PisteBalle t : pistesBalles) {
            if (t.lastStep == compteurStep) continue;
            double dx = t.lastX - x, dy = t.lastY - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { meilleureDistance2 = d2; meilleur = t; }
        }
        double maxD2 = DISTANCE_ASSOCIATION_PISTE_BALLE * DISTANCE_ASSOCIATION_PISTE_BALLE;
        if (meilleur != null && meilleureDistance2 <= maxD2) { meilleur.update(x, y, compteurStep); return meilleur; }
        PisteBalle created = new PisteBalle(x, y, compteurStep);
        pistesBalles.add(created);
        return created;
    }

    // Nettoie les pistes de balles qui n'ont pas ete vues depuis un moment
    private void nettoyerPistesBallesExpirees() {
        pistesBalles.removeIf(t -> compteurStep - t.lastStep > AGE_MAX_PISTE_BALLE);
    }

    // Repart de toutes les pistes de balles pour reconstruire la table partagee des projectiles en gardant seulement ce qui semble encore pertinent
    private void synchroniserListeProjectilesDepuisPistes() {
        listeProjectiles.removeIf(p -> p.source == TypeSource.RADAR);
        for (PisteBalle t : pistesBalles) {
            if (t.alias == AliasProjectile.ALLIE) continue;
            mettreAJourProjectile(t.lastX, t.lastY, TypeSource.RADAR);
        }
    }

    // Met a jour la memoire des ennemis
    // Si un point proche existe deja, on le lisse sinon on en cree un nouveau
    protected boolean mettreAJourEnnemI(double x, double y, boolean allowCreate, TypeSource source) {
        return mettreAJourEnnemI(x, y, allowCreate, source, false);
    }

    // Meme logique que plus haut, mais avec l'information supplementaire"detecte directement" ou "prediction"
    protected boolean mettreAJourEnnemI(double x, double y, boolean allowCreate, TypeSource source, boolean predit) {
        x = borner(x, CARTE_MIN_X, CARTE_MAX_X);
        y = borner(y, CARTE_MIN_Y, CARTE_MAX_Y);
        int meilleurIndex = -1;
        double meilleureDistance2 = Double.MAX_VALUE;
        for (int i = 0; i < listeEnnemis.size(); i++) {
            PointSuivi p = listeEnnemis.get(i);
            double dx = p.x - x, dy = p.y - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { meilleureDistance2 = d2; meilleurIndex = i; }
        }
        double distanceFusion2 = DISTANCE_FUSION_ROBOT * DISTANCE_FUSION_ROBOT;
        if (meilleurIndex >= 0 && meilleureDistance2 <= distanceFusion2) {
            PointSuivi p = listeEnnemis.get(meilleurIndex);
            if (doitIgnorerEntrant(p, source)) return false;
            double facteurConservation = (source == TypeSource.RADAR) ? 0.80 : 0.88;
            p.x = facteurConservation * p.x + (1.0 - facteurConservation) * x;
            p.y = facteurConservation * p.y + (1.0 - facteurConservation) * y;
            p.predit = p.predit && predit;
            p.source = source;
            p.dernierStepMiseAJour = compteurStep;
            return false;
        }
        if (!allowCreate || listeEnnemis.size() >= MAX_PISTES_ENNEMIES) return false;
        listeEnnemis.add(new PointSuivi(x, y, source, compteurStep, predit));
        return true;
    }

    // Meme principe que pour les ennemis, mais applique aux projectiles
    protected void mettreAJourProjectile(double x, double y, TypeSource source) {
        x = borner(x, CARTE_MIN_X, CARTE_MAX_X);
        y = borner(y, CARTE_MIN_Y, CARTE_MAX_Y);
        int meilleurIndex = -1;
        double meilleureDistance2 = Double.MAX_VALUE;
        for (int i = 0; i < listeProjectiles.size(); i++) {
            PointSuivi p = listeProjectiles.get(i);
            double dx = p.x - x, dy = p.y - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) {
                meilleureDistance2 = d2;
                meilleurIndex = i;
            }
        }
        double distanceFusion2 = DISTANCE_ASSOCIATION_PISTE_BALLE * DISTANCE_ASSOCIATION_PISTE_BALLE;
        if (meilleurIndex >= 0 && meilleureDistance2 <= distanceFusion2) {
            PointSuivi p = listeProjectiles.get(meilleurIndex);
            if (doitIgnorerEntrant(p, source)) return;
            double facteurConservation = (source == TypeSource.RADAR) ? 0.80 : 0.88;
            p.x = facteurConservation * p.x + (1.0 - facteurConservation) * x;
            p.y = facteurConservation * p.y + (1.0 - facteurConservation) * y;
            p.source = source;
            p.dernierStepMiseAJour = compteurStep;
            return;
        }
        listeProjectiles.add(new PointSuivi(x, y, source, compteurStep));
    }

    // Quand on a dectete qlq chose dans le radar recente alors le broadcast perd la priorité
    private boolean doitIgnorerEntrant(PointSuivi existing, TypeSource incoming) {
        return existing.source == TypeSource.RADAR && incoming  == TypeSource.DIFFUSION && compteurStep - existing.dernierStepMiseAJour <= 5;
    }

    // Supprime les infos trop anciennes, sauf si on prefere garde un souvenir lointain d'un ennemi encore dangereux.
    private void nettoyerPointsDiffusionPerimes() {
        listeEnnemis.removeIf(p -> p.source == TypeSource.RADAR && compteurStep - p.dernierStepMiseAJour > TTL_ENNEMI_RADAR && !conserverMemoireEnnemLointain(p));
        listeEnnemis.removeIf(p -> p.source == TypeSource.DIFFUSION && compteurStep - p.dernierStepMiseAJour > TTL_DIFFUSION && !conserverMemoireEnnemLointain(p));
        listeProjectiles.removeIf(p -> p.source == TypeSource.RADAR && compteurStep - p.dernierStepMiseAJour > TTL_BALLE_RADAR);
        listeProjectiles.removeIf(p -> p.source == TypeSource.DIFFUSION && compteurStep - p.dernierStepMiseAJour > TTL_DIFFUSION);
        indicesTirRecents.removeIf(h -> compteurStep - h.dernierStepMiseAJour > TTL_INDICE_TIR);
    }

    //on garde une trace d'un ennemi eloigne tant qu'il reste assez loin pour ne pas perturber le comportement du robot
    private boolean conserverMemoireEnnemLointain(PointSuivi p) {
        double dx = p.x - monX;
        double dy = p.y - monY;
        double keepD2 = DISTANCE_MEMOIRE_ENNEMI_LOIN * DISTANCE_MEMOIRE_ENNEMI_LOIN;
        return dx * dx + dy * dy > keepD2;
    }

    // on lock l'enemi pour ne pas changer de cible trop brutalement d'un step a l'autre
    protected void mettreAJourVerrouEnnemDepuisRadar(ArrayList<PointSuivi> ennemisRadarCeStep) {
        if (ennemisRadarCeStep.isEmpty()) return;
        if (!aEnnemVerrouille || compteurStep - dernierStepVerrouEnnemi > STEPS_MAINTIEN_VERROU) {
            PointSuivi first = ennemisRadarCeStep.get(0);
            aEnnemVerrouille = true;
            xEnnemVerrouille = first.x;
            yEnnemVerrouille = first.y;
            dernierStepVerrouEnnemi = compteurStep;
            return;
        }
        double maxMatchDist2 = 320.0 * 320.0;
        PointSuivi closest = null;
        double closestDist2 = Double.MAX_VALUE;
        for (PointSuivi p : ennemisRadarCeStep) {
            double dx = p.x - xEnnemVerrouille, dy = p.y - yEnnemVerrouille;
            double d2 = dx * dx + dy * dy;
            if (d2 < closestDist2) {
                closestDist2 = d2;
                closest = p;
            }
        }
        if (closest != null && closestDist2 <= maxMatchDist2) {
            xEnnemVerrouille = closest.x; yEnnemVerrouille = closest.y; dernierStepVerrouEnnemi = compteurStep;
            return;
        }
        PointSuivi first = ennemisRadarCeStep.get(0);
        xEnnemVerrouille = first.x; yEnnemVerrouille = first.y; dernierStepVerrouEnnemi = compteurStep;
    }

    // on selection et on lock l'enemi

    /* Ordre de priorite des cibles:
     * 1. ennemi verrouille ou vu directement
     * 2. information diffusee par les allies
     * 3. ennemi devine a partir d'un projectile
     * 4. projectiles eux-memes si on n'a rien de mieux */
    protected CandidatCible selectionnerCibleParPriorite() {
        PointSuivi radarEnemyThisStep = selectionnerLePlusProche(ennemisRadarCeStep, TypeSource.RADAR);
        if (radarEnemyThisStep != null) return new CandidatCible(radarEnemyThisStep, 0);
        if (aEnnemVerrouille && compteurStep - dernierStepVerrouEnnemi <= STEPS_MAINTIEN_VERROU) return new CandidatCible(new PointSuivi(xEnnemVerrouille, yEnnemVerrouille, TypeSource.RADAR, dernierStepVerrouEnnemi, false), 0);
        PointSuivi radarEnemy = selectionnerEnnemLePlusProche(TypeSource.RADAR, false);
        if (radarEnemy != null) return new CandidatCible(radarEnemy, 0);
        PointSuivi broadcastEnemy = selectionnerEnnemLePlusProche(TypeSource.DIFFUSION, false);
        if (broadcastEnemy != null) return new CandidatCible(broadcastEnemy, 1);
        PointSuivi predictedEnemy = selectionnerEnnemPreditLePlusProche();
        if (predictedEnemy != null) return new CandidatCible(predictedEnemy, 2);
        PointSuivi radarBullet = selectionnerLePlusProche(listeProjectiles, TypeSource.RADAR);
        if (radarBullet != null) return new CandidatCible(radarBullet, 3);
        PointSuivi broadcastBullet = selectionnerLePlusProche(listeProjectiles, TypeSource.DIFFUSION);
        if (broadcastBullet != null) return new CandidatCible(broadcastBullet, 4);
        return null;
    }

    // il retourne l'ennemi le plus proche (radar ou diffusion) et selon le statut predit/non predit
    protected PointSuivi selectionnerEnnemLePlusProche(TypeSource source, boolean predit) {
        PointSuivi meilleur = null;
        double meilleureDistance2 = Double.MAX_VALUE;
        for (PointSuivi p : listeEnnemis) {
            if (p.source != source) continue;
            if (p.predit != predit) continue;
            double dx = p.x - monX, dy = p.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { meilleureDistance2 = d2; meilleur = p; }
        }
        return meilleur;
    }

    // Il cherche le meilleur ennemi parmi toutes les predictions disponibles
    protected PointSuivi selectionnerEnnemPreditLePlusProche() {
        PointSuivi meilleur = null;
        double meilleureDistance2 = Double.MAX_VALUE;
        for (PointSuivi p : listeEnnemis) {
            if (!p.predit) continue;
            double dx = p.x - monX, dy = p.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) { meilleureDistance2 = d2; meilleur = p; }
        }
        return meilleur;
    }

    //Evite que la cible saute sans arret: si on a deja une cible recente, on la garde un peu et on lisse ses variations
    protected CandidatCible appliquerVerrouCible(CandidatCible candidat) {
        if (candidat == null) {
            if (aVerrouCible && compteurStep <= verrouilleJusquauStep) {
                PointSuivi p = new PointSuivi(xVerrouCible, yVerrouCible, TypeSource.RADAR, compteurStep, false);
                return new CandidatCible(p, prioriteVerrouCible);
            }
            aVerrouCible = false;
            return null;
        }
        if (!aVerrouCible) {
            definirVerrouCible(candidat);
            return candidat;
        }
        if (compteurStep <= verrouilleJusquauStep) {
            double facteuraLissage = 0.85;
            if (candidat.priorite + 1 < prioriteVerrouCible) {
                definirVerrouCible(candidat);
                return candidat;
            }
            if (candidat.priorite == prioriteVerrouCible) {
                xVerrouCible = facteuraLissage * xVerrouCible + (1.0 - facteuraLissage) * candidat.point.x;
                yVerrouCible = facteuraLissage * yVerrouCible + (1.0 - facteuraLissage) * candidat.point.y;
            }
            PointSuivi p = new PointSuivi(xVerrouCible, yVerrouCible, TypeSource.RADAR, compteurStep, false);
            return new CandidatCible(p, prioriteVerrouCible);
        }
        definirVerrouCible(candidat);
        return candidat;
    }

    // Enregistre la cible courante comme nouvelle cible
    private void definirVerrouCible(CandidatCible candidat) {
        aVerrouCible = true;
        xVerrouCible = candidat.point.x;
        yVerrouCible = candidat.point.y;
        prioriteVerrouCible = candidat.priorite;
        verrouilleJusquauStep = compteurStep + 10;
    }

    // Retourne le point le plus proche dans une liste donnee
    protected PointSuivi selectionnerLePlusProche(ArrayList<PointSuivi> points, TypeSource source) {
        PointSuivi meilleur = null;
        double meilleureDistance2 = Double.MAX_VALUE;
        for (PointSuivi p : points) {
            if (p.source != source) continue;
            double dx = p.x - monX, dy = p.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 < meilleureDistance2) {
                meilleureDistance2 = d2;
                meilleur = p;
            }
        }
        return meilleur;
    }

    // Deplacement principal: on essaie d'aller vers la cible,
    // mais on garde toujours un oeil sur les obstacles et les blocages
    protected void seDeplacerVers(double targetX, double targetY) {
        if (executerDeblocageDroite()) return;
        if (stepsRotationForcee > 0) {
            stepTurn(dirRotationForcee);
            stepsRotationForcee--;
            return;
        }
        double directionSouhaitee = Math.atan2(targetY - monY, targetX - monX);
        double delta = normaliserAngle(directionSouhaitee - getHeading());
        double deltaAbsolu = Math.abs(delta);
        double seuilAlignement = monAngleTourParStep * MULTIPLICATEUR_SEUIL_ALIGNEMENT;
        if (!modeReorientation && deltaAbsolu > ANGLE_CONE_AVANT) modeReorientation = true;
        if (modeReorientation  && deltaAbsolu <= ANGLE_SORTIE_REORIENTATION)  modeReorientation = false;
        if (!modeReorientation || deltaAbsolu <= seuilAlignement) {
            IFrontSensorResult.Types typeFront = detectFront().getObjectType();
            if (typeFront == IFrontSensorResult.Types.NOTHING) {
                stepsBlockeAvant = 0;
                avancerAvecOdometrie();
                return;
            }
            stepsBlockeAvant++;
            if (stepsBlockeAvant >= SEUIL_BLOCAGE_AVANT) {
                modeDeblocageDroite = true;
                stepsDeblocageDroiteRestants = MAX_STEPS_DEBLOCAGE_DROITE;
                stepsBlockeAvant = 0;
                reculerAvecOdometrie();
                stepTurn(Parameters.Direction.RIGHT);
            } else {
                stepTurn(Math.random() < 0.5 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
            }
            return;
        }
        stepsBlockeAvant = 0;
        if (delta > 0.0) stepTurn(Parameters.Direction.RIGHT);
        else stepTurn(Parameters.Direction.LEFT);
    }

    // Strategie simple quand on est coince: on force un petit detour a droite jusqu'a retrouver un chemin libre
    protected boolean executerDeblocageDroite() {
        if (!modeDeblocageDroite) return false;
        if (stepsDeblocageDroiteRestants <= 0) { modeDeblocageDroite = false; return false; }
        IFrontSensorResult.Types typeFront = detectFront().getObjectType();
        if (typeFront != IFrontSensorResult.Types.NOTHING && typeFront != IFrontSensorResult.Types.BULLET) {
            stepsDeblocageDroiteRestants--;
            reculerAvecOdometrie();
            stepTurn(Parameters.Direction.RIGHT);
            tentativeAvanceStep = false;
            return true;
        }
        stepsDeblocageDroiteRestants--;
        if ((stepsDeblocageDroiteRestants % PERIODE_ZIGZAG_DEBLOCAGE) == 0) {
            stepTurn(Parameters.Direction.RIGHT);
            tentativeAvanceStep = false;
            return true;
        }
        avancerAvecOdometrie();
        return true;
    }

    // Avance en restant conservateur: on n'actualise l'odometrie qu'apres
    // l'ordre de mouvement, et seulement si le passage avant semble libre.
    protected void avancerAvecOdometrie() {
        IFrontSensorResult.Types typeFront = detectFront().getObjectType();
        boolean passageAvantLibre = typeFront == IFrontSensorResult.Types.NOTHING
                || typeFront == IFrontSensorResult.Types.BULLET;
        tentativeAvanceStep = true;
        move();
        if (!passageAvantLibre) {
            return;
        }
        monX += maVitesse * Math.cos(getHeading());
        monY += maVitesse * Math.sin(getHeading());
        monX = borner(monX, CARTE_MIN_X, CARTE_MAX_X);
        monY = borner(monY, CARTE_MIN_Y, CARTE_MAX_Y);
        mettreAJourPositionAllie(idInstance, monX, monY);
    }

    // Recul odometrique: on applique la correction de position apres le mouvement.
    protected void reculerAvecOdometrie() {
        moveBack();
        monX -= maVitesse * Math.cos(getHeading());
        monY -= maVitesse * Math.sin(getHeading());
        monX = borner(monX, CARTE_MIN_X, CARTE_MAX_X);
        monY = borner(monY, CARTE_MIN_Y, CARTE_MAX_Y);
        mettreAJourPositionAllie(idInstance, monX, monY);
        tentativeAvanceStep = false;
    }

    // Friendly fire 
    // Regarde simplement ce qui est devant nous pour eviter un tir allier
    protected boolean allieDevant() {
        IFrontSensorResult.Types typeFront = detectFront().getObjectType();
        return typeFront == IFrontSensorResult.Types.TeamMainBot || typeFront == IFrontSensorResult.Types.TeamSecondaryBot;
    }

    // Verifie si un allie connu risque de se retrouver sur notre ligne de tir
    protected boolean allieDansLigneDeViseeDepuisListe(double directionTir, double distanceTir) {
        double distanceMax2 = Parameters.bulletRange * Parameters.bulletRange;
        double rayonHitboxAllie = RAYON_MAX_HITBOX_ALLIE + MARGE_EXTRA_HITBOX_FRATRICIDE;
        for (int i = 0; i < positionsAllies.size(); i++) {
            int idEmetteur = i + 1;
            if (idEmetteur == idInstance) continue;
            PositionAllie ap = positionsAllies.get(i);
            if (!ap.known || compteurStep - ap.dernierStepMiseAJour > TTL_POSITION_ALLIE) continue;
            double dx = ap.x - monX, dy = ap.y - monY;
            double d2 = dx * dx + dy * dy;
            if (d2 > distanceMax2 || d2 < 1.0) continue;
            double distanceAllie = Math.sqrt(d2);
            if (distanceAllie >= distanceTir + rayonHitboxAllie + EXTENSION_ARRIERE_TIR_FRATRICIDE) continue;
            double directionAllie = Math.atan2(dy, dx);
            double angleBlockageGeometrique = Math.asin(Math.min(1.0, rayonHitboxAllie / Math.max(distanceAllie, 1.0)));
            double angleBlockageConservatif = Math.max(ANGLE_BLOCAGE_TIR_FRATRICIDE, angleBlockageGeometrique);
            if (Math.abs(normaliserAngle(directionAllie - directionTir)) <= angleBlockageConservatif) return true;
        }
        return false;
    }

    // Meme controle mais cette fois avec les allies visibles directement au radar
    protected boolean allieDansLigneDeViseeDepuisRadar(
            ArrayList<IRadarResult> radar, double directionTir, double distanceTir) {
        double rayonHitboxAllie = RAYON_MAX_HITBOX_ALLIE + MARGE_EXTRA_HITBOX_FRATRICIDE + MARGE_EXTRA_RADAR_FRATRICIDE;
        for (IRadarResult r : radar) {
            IRadarResult.Types t = r.getObjectType();
            if (t != IRadarResult.Types.TeamMainBot && t != IRadarResult.Types.TeamSecondaryBot) continue;
            double distanceAllie = r.getObjectDistance();
            if (distanceAllie < 1.0 || distanceAllie > Parameters.bulletRange) continue;
            if (distanceAllie >= distanceTir + rayonHitboxAllie + EXTENSION_ARRIERE_TIR_FRATRICIDE) continue;
            double directionAllie = r.getObjectDirection();
            double angleBlockageGeometrique  = Math.asin(Math.min(1.0, rayonHitboxAllie / Math.max(distanceAllie, 1.0)));
            double angleBlockageConservatif = Math.max(ANGLE_BLOCAGE_TIR_FRATRICIDE, angleBlockageGeometrique);
            if (Math.abs(normaliserAngle(directionAllie - directionTir)) <= angleBlockageConservatif) return true;
        }
        return false;
    }

    // communication d'information

    // Recupere les messages d'equipe et les traites les info : positions d'alliés, tirs recents, ennemis detectes, etc
    protected void traiterMessagesEntrants() {
        ArrayList<String> messages = fetchAllMessages();
        for (String message : messages) {
            if (message == null || message.isEmpty()) continue;
            String[] parts = message.split("\\|");
            if (parts.length != 6 || !PREFIXE_MSG.equals(parts[0])) continue;
            int idEmetteur;
            try {
                idEmetteur = Integer.parseInt(parts[5]);
            } catch (NumberFormatException ex) { continue; }
            if (idEmetteur == idInstance) continue;
            double x, y;
            try {
                x = Double.parseDouble(parts[2]);
                y = Double.parseDouble(parts[3]);
            } catch (NumberFormatException ex) { continue; }
            if (TYPE_MSG_ENNEMI.equals(parts[1])) {
                mettreAJourEnnemI(x, y, true, TypeSource.DIFFUSION, false);
            }
            else if (TYPE_MSG_ENNEMI_PREDIT.equals(parts[1])) { mettreAJourEnnemI(x, y, true, TypeSource.DIFFUSION, true); }
            else if (TYPE_MSG_BALLE.equals(parts[1])) { mettreAJourProjectile(x, y, TypeSource.DIFFUSION); }
            else if (TYPE_MSG_TIR.equals(parts[1])) {
                double directionSource;
                try { directionSource = Double.parseDouble(parts[4]); }
                catch (NumberFormatException ex) { directionSource = Double.NaN; }
                if (!Double.isNaN(directionSource)) ajouterIndiceTirRecent(idEmetteur, x, y, directionSource);
            } else if (TYPE_MSG_POS.equals(parts[1])) { mettreAJourPositionAllie(idEmetteur, x, y); }
        }
    }

    // Quand on tire, on garde la trace localement et on previent les allies
    protected void tirerAvecIndice(double directionTir) {
        fire(directionTir);
        dernierStepTir = compteurStep;
        derniereDirTir = directionTir;
        diffuserDetection(TYPE_MSG_TIR, monX, monY, directionTir);
    }

    // Envoie un message court et standardise sur le canal d'equipe
    protected void diffuserDetection(String type, double x, double y, double directionSource) {
        String msg = PREFIXE_MSG + "|" + type + "|" + String.valueOf(x) + "|" + String.valueOf(y) + "|" + (Double.isNaN(directionSource) ? "NA" : String.valueOf(directionSource)) + "|" + idInstance;
        broadcast(msg);
    }

    // Met a jour la position d'un allie dans la memoire commune
    protected void mettreAJourPositionAllie(int idEmetteur, double x, double y) {
        if (idEmetteur < 1 || idEmetteur > NOMBRE_ROBOTS_EQUIPE) return;
        PositionAllie ap = positionsAllies.get(idEmetteur - 1);
        ap.x = borner(x, CARTE_MIN_X, CARTE_MAX_X);
        ap.y = borner(y, CARTE_MIN_Y, CARTE_MAX_Y);
        ap.dernierStepMiseAJour = compteurStep;
        ap.known = true;
    }

    // Petit outil de securite pour garder une valeur dans un intervalle donne
    protected double borner(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Ramene n'importe quel angle dans l'intervalle ]-PI, PI]
    protected double normaliserAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    // Affiche un petit statut lisible dans l'interface pour savoir ce que fait le robot
    protected void publierStatut(String state) {
        sendLogMessage(getClass().getSimpleName() + "#" + idInstance + " " + state);
    }

    // Donne un id unique a chaque robot pour que les messages restent interpretables.
    private static synchronized int acquerirIdInstance() { 
        return PROCHAIN_ID_INSTANCE++; 
    }

    // classe interne partager
    protected static class PointSuivi {
        double x, y; TypeSource source; int dernierStepMiseAJour; boolean predit;
        PointSuivi(double x, double y, TypeSource source, int dernierStepMiseAJour) {
            this(x, y, source, dernierStepMiseAJour, false);
        }
        PointSuivi(double x, double y, TypeSource source, int dernierStepMiseAJour, boolean predit) {
            this.x = x; this.y = y; this.source = source; this.dernierStepMiseAJour = dernierStepMiseAJour; this.predit = predit;
        }
    }

    protected static class PredictionSource {
        double x, y, directionSource; AliasProjectile alias;
        PredictionSource(double x, double y, double directionSource, AliasProjectile alias) {
            this.x = x; this.y = y; this.directionSource = directionSource; this.alias = alias;
        }
    }

    protected static class IndiceTirRecent {
        int idEmetteur, dernierStepMiseAJour; double x, y, direction;
        IndiceTirRecent(int idEmetteur, double x, double y, double direction, int dernierStepMiseAJour) {
            this.idEmetteur = idEmetteur; this.x = x; this.y = y; this.direction = direction; this.dernierStepMiseAJour = dernierStepMiseAJour;
        }
    }

    protected static class CandidatCible {
        PointSuivi point; int priorite;
        CandidatCible(PointSuivi point, int priorite) { this.point = point; this.priorite = priorite; }
    }

    protected static class PlanDepart {
        double capSeparation;
        int ticksDroite;
        PlanDepart(double capSeparation, int ticksDroite) {
            this.capSeparation = capSeparation;
            this.ticksDroite = ticksDroite;
        }
    }

    protected static class PisteBalle {
        double prevX, prevY, lastX, lastY; int lastStep; boolean aPrecedent; AliasProjectile alias;
        PisteBalle(double x, double y, int step) {
            lastX = x; lastY = y; lastStep = step; alias = AliasProjectile.INCONNU;
        }
        void update(double x, double y, int step) {
            prevX = lastX; prevY = lastY; lastX = x; lastY = y; lastStep = step; aPrecedent = true;
        }
    }

    protected static class PositionAllie {
        double x, y; int dernierStepMiseAJour; boolean known;
    }
}