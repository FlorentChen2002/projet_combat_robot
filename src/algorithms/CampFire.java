package algorithms;

import java.util.ArrayList;
import java.util.Random;

import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;

public class CampFire extends Brain {
  private boolean turnTask, turnRight, endMove, taskOne;
  private double endTaskDirection;
  private int endTaskCounter, id, latence;
  private static IFrontSensorResult.Types WALL = IFrontSensorResult.Types.WALL;
  private Random gen;
  private String algoName;

  // 🔥 AJOUT : position du robot
  private double myX, myY, mySpeed;
  private boolean moveInProgress;
  private static int IndexMainBotA = 0;
	private static int IndexMainBotB = 0;

  public CampFire() { super(); gen = new Random(); }

  public void activate() {
    algoName = getClass().getSimpleName();
    latence = -1;
    turnTask = true;
    endMove = false;
    taskOne = true;
    endTaskDirection = getHeading() + 0.5 * Math.PI;

    // 🔥 Initialisation position (approximation selon équipe)
    if (Math.cos(getHeading() - Parameters.EAST) > 0) {
				IndexMainBotA++;
				if (IndexMainBotA == 1){
					myX = Parameters.teamAMainBot1InitX;
					myY = Parameters.teamAMainBot1InitY;
				} else if (IndexMainBotA == 2){
					myX = Parameters.teamAMainBot2InitX;
					myY = Parameters.teamAMainBot2InitY;
				} else {
					myX = Parameters.teamAMainBot3InitX;
					myY = Parameters.teamAMainBot3InitY;
				}
				mySpeed = Parameters.teamAMainBotSpeed;
    } else {
      IndexMainBotB++;
      if (IndexMainBotB == 1){
        myX = Parameters.teamBMainBot1InitX;
        myY = Parameters.teamBMainBot1InitY;
      } else if (IndexMainBotB == 2){
        myX = Parameters.teamBMainBot2InitX;
        myY = Parameters.teamBMainBot2InitY;
      } else {
        myX = Parameters.teamBMainBot3InitX;
        myY = Parameters.teamBMainBot3InitY;
      }
      mySpeed = Parameters.teamBMainBotSpeed;
    }

    moveInProgress = false;

    stepTurn(Parameters.Direction.RIGHT);
    sendLogMessage("Rocking and rolling.");
  }

  public void step() {
      if (getHealth() <= 0) {
        sendLogMessage("I'm dead.");
        return;
      }

      // 🔥 Mise à jour position
      updateOdometry();


      if (endMove) {
        sendLogMessage("Camping point. Task one complete.");
        campFire();
        return;
      }

      if (turnTask) {
        if (isHeading(endTaskDirection)) {
          turnTask = false;
          if (taskOne) endTaskCounter = 700;
          else if (id == 1) endTaskCounter = 400;
          else endTaskCounter = 250;

          move();
          moveInProgress = true;
        } else {
          if (taskOne) stepTurn(Parameters.Direction.RIGHT);
          else stepTurn(Parameters.Direction.LEFT);
        }
        return;
      }

      if (endTaskCounter > 0) {
        endTaskCounter--;
        move();
        moveInProgress = true;
        return;
      } else {
        if (taskOne) taskOne = false;
        else {
          endMove = true;
          return;
        }

        id = 0;
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult r : radarResults)
          if (r.getObjectType() == IRadarResult.Types.TeamMainBot ||
              r.getObjectType() == IRadarResult.Types.TeamSecondaryBot)
            id++;

        if (id == 2) id = 3;
        else if (id == 3) id = 2;

        if (id == 3) {
          endMove = true;
        } else {
          turnTask = true;
          endTaskDirection = getHeading() - 0.5 * Math.PI;
          stepTurn(Parameters.Direction.LEFT);
        }
        return;
      }
    }

    private void campFire() {
      ArrayList<IRadarResult> radarResults = detectRadar();
      int enemyFighters = 0, enemyPatrols = 0;
      double enemyDirection = 0;
      for (IRadarResult r : radarResults) {
        if (r.getObjectType() == IRadarResult.Types.BULLET) {
          printRadarCoordinate("projectile", r);
        }
        if (r.getObjectType() == IRadarResult.Types.OpponentMainBot) {
          enemyFighters++;
          enemyDirection = r.getObjectDirection();
          printRadarCoordinate("enemy-main", r);
          continue;
        }
        if (r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
          if (enemyFighters == 0) enemyDirection = r.getObjectDirection();
          enemyPatrols++;
          printRadarCoordinate("enemy-secondary", r);
        }
      }

      if (latence < 0) {
        if (enemyFighters + enemyPatrols == 0) {
          if (id == 1) fire(Math.PI * (0.98 + 0.04 * gen.nextDouble()));
          if (id == 2) fire(Math.PI * (0.60 + 0.4 * gen.nextDouble()));
          if (id == 3) fire(Math.PI * (0.60 + 0.2 * gen.nextDouble()));
          latence = 21;
          return;
        }
        fire(enemyDirection);
        latence = 21;
        return;
      } else latence--;
    }

    // 🔥 Conversion radar -> coordonnées relatives
    private void printRadarCoordinate(String label, IRadarResult r) {
      double relX = r.getObjectDistance() * Math.cos(r.getObjectDirection());
      double relY = r.getObjectDistance() * Math.sin(r.getObjectDirection());
      System.out.println("[" + algoName + "] " + label + " relCoord=(" + (int)relX + ", " + (int)relY + ") dist=" + (int)r.getObjectDistance());
    }

    // 🔥 Mise à jour position
    private void updateOdometry() {
      if (!moveInProgress) return;
      myX += mySpeed * Math.cos(getHeading());
      myY += mySpeed * Math.sin(getHeading());
      moveInProgress = false;
    }

    private boolean isHeading(double dir) {
      return Math.abs(Math.sin(getHeading() - dir)) < Parameters.teamBSecondaryBotStepTurnAngle;
  }
}