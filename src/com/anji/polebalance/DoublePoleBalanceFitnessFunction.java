/*
 * Copyright (C) 2004 Derek James and Philip Tucker
 * 
 * This file is part of ANJI (Another NEAT Java Implementation).
 * 
 * ANJI is free software; you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with this program; if
 * not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 * 
 * created by Derek James on Jul 5, 2005
 */
package com.anji.polebalance;

import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.nio.DoubleBuffer;

import org.apache.log4j.Logger;
import org.jgap.BulkFitnessFunction;
import org.jgap.Chromosome;

import com.anji.imaging.IdentifyImageFitnessFunction;
import com.anji.integration.Activator;
import com.anji.integration.ActivatorTranscriber;
import com.anji.util.Arrays;
import com.anji.util.Configurable;
import com.anji.util.Properties;
import com.anji.util.Randomizer;
import dk.itu.gaer.Board;

import dk.itu.gaer.Pacman;
import java.util.logging.Level;

/**
 * This code is a port from Colin Green's SharpNEAT pole balancing code, which
 * in turn is a port from Ken Stanley's NEAT code.
 *
 * @author Derek James
 */
public class DoublePoleBalanceFitnessFunction implements BulkFitnessFunction, Configurable {

    private final static String TRACK_LENGTH_KEY = "polebalance.track.length";

    private final static String TIMESTEPS_KEY = "polebalance.timesteps";

    private final static String NUM_TRIALS_KEY = "polebalance.trials";

    private final static String ANGLE_THRESHOLD_KEY = "polebalance.angle.threshold";

    private final static String INPUT_VELOCITY_KEY = "polebalance.input.velocities";

    private final static String POLE_1_LENGTH_KEY = "pole.1.length";

    private final static String POLE_2_LENGTH_KEY = "pole.2.length";

    private final static String START_POLE_ANGLE_1_KEY = "polebalance.pole.angle.start.1";

    private final static String START_POLE_ANGLE_2_KEY = "polebalance.pole.angle.start.2";

    private final static String START_POLE_ANGLE_RANDOM_KEY = "polebalance.pole.angle.start.random";

    private final static String PENALIZE_FOR_ENERGY_USE_KEY = "penalize.for.energy.use";

    private final static String PENALIZE_OSCILLATIONS_KEY = "penalize.oscillations";

// Some useful physical model constants.
    private static final double GRAVITY = -9.8;

    private static final double MASSCART = 1.0;

    private static final double FORCE_MAG = 10.0;

    /**
     * seconds between state updates
     */
    private static final double TIME_DELTA = 0.01;

    private static final double FOURTHIRDS = 4.0 / 3.0;

    private static final double MUP = 0.000002;

    /**
     * 0.0174532; 2pi/360
     */
    private static final double ONE_DEGREE = Math.PI / 180.0;

    /**
     * 0.1047192
     */
    private static final double SIX_DEGREES = Math.PI / 30.0;

    /**
     * 0.2094384;
     */
    private static final double TWELVE_DEGREES = Math.PI / 15.0;

    /**
     * 0.3141592;
     */
    private static final double EIGHTEEN_DEGREES = Math.PI / 10.0;

    /**
     * 0.4188790;
     */
    private static final double TWENTYFOUR_DEGREES = Math.PI / 7.5;

    /**
     * 0.628329;
     */
    private static final double THIRTYSIX_DEGREES = Math.PI / 5.0;

    /**
     * 0.87266;
     */
    private static final double FIFTY_DEGREES = Math.PI / 3.6;

    /**
     * 1.256637;
     */
    private static final double SEVENTYTWO_DEGREES = Math.PI / 2.5;

    private PoleBalanceDisplay display = null;

    private final static double DEFAULT_TRACK_LENGTH = 4.8;

    private double trackLength = DEFAULT_TRACK_LENGTH;

    private double trackLengthHalfed;

    private final static int DEFAULT_TIMESTEPS = 10000;

    private int maxTimesteps = DEFAULT_TIMESTEPS;

    private final static int DEFAULT_NUM_TRIALS = 10;

    private int numTrials = DEFAULT_NUM_TRIALS;

    private double poleAngleThreshold = THIRTYSIX_DEGREES;

    private final static Logger logger = Logger.getLogger(DoublePoleBalanceFitnessFunction.class);

    private ActivatorTranscriber factory;

    private boolean doInputVelocities = true;

    private double poleLength1 = 0.5;

    private double poleMass1 = 0.1;

    private double poleLength2 = 0.05;

    private double poleMass2 = 0.01;

    private double startPoleAngle1 = ONE_DEGREE;

    private double startPoleAngle2 = 0;

    private boolean startPoleAngleRandom = false;

    private boolean penalizeEnergyUse = false;

    private boolean penalizeOscillations = false;

    private Random rand;

    private boolean showGame = false;

    private void setTrackLength(double aTrackLength) {
        trackLength = aTrackLength;
        trackLengthHalfed = trackLength / 2;
    }

    /**
     * @see com.anji.util.Configurable#init(com.anji.util.Properties)
     */
    public void init(Properties props) throws Exception {
        try {
            factory = (ActivatorTranscriber) props.singletonObjectProperty(ActivatorTranscriber.class);
            setTrackLength(props.getDoubleProperty(TRACK_LENGTH_KEY, DEFAULT_TRACK_LENGTH));
            maxTimesteps = props.getIntProperty(TIMESTEPS_KEY, DEFAULT_TIMESTEPS);
            numTrials = props.getIntProperty(NUM_TRIALS_KEY, DEFAULT_NUM_TRIALS);
            poleAngleThreshold = props.getDoubleProperty(ANGLE_THRESHOLD_KEY, THIRTYSIX_DEGREES);
            doInputVelocities = props.getBooleanProperty(INPUT_VELOCITY_KEY, true);
            poleLength1 = (props.getDoubleProperty(POLE_1_LENGTH_KEY, 0.5) / 2);
            poleMass1 = (poleLength1 / 5);
            poleLength2 = (props.getDoubleProperty(POLE_2_LENGTH_KEY, 0.05) / 2);
            poleMass2 = (poleLength2 / 5);
            startPoleAngle1 = props.getDoubleProperty(START_POLE_ANGLE_1_KEY, ONE_DEGREE);
            startPoleAngle2 = props.getDoubleProperty(START_POLE_ANGLE_2_KEY, 0);
            startPoleAngleRandom = props.getBooleanProperty(START_POLE_ANGLE_RANDOM_KEY, false);
            penalizeEnergyUse = props.getBooleanProperty(PENALIZE_FOR_ENERGY_USE_KEY, false);
            penalizeOscillations = props.getBooleanProperty(PENALIZE_OSCILLATIONS_KEY, false);
            Randomizer randomizer = (Randomizer) props.singletonObjectProperty(Randomizer.class);
            rand = randomizer.getRand();
        } catch (Exception e) {
            throw new IllegalArgumentException("invalid properties: " + e.getClass().toString() + ": "
                    + e.getMessage());
        }
    }

    /**
     * @see org.jgap.BulkFitnessFunction#evaluate(java.util.List)
     * @see IdentifyImageFitnessFunction#evaluate(Chromosome)
     */
    public void evaluate(List genotypes) {

        // evaluate each chromosome
        Iterator it = genotypes.iterator();
        while (it.hasNext()) {
            Chromosome c = (Chromosome) it.next();
            evaluate(c);
        }
    }

    /**
     * Evaluate chromosome and set fitness.
     *
     * @param c
     */
    public void evaluate(Chromosome c) {
        try {
            Activator activator = factory.newActivator(c);

            // calculate fitness, sum of multiple trials
            int fitness = 0;
            for (int i = 0; i < numTrials; i++) {
                fitness += singleTrial(activator);
                if (showGame) {
                    break;
                }
            }
            c.setFitnessValue(fitness);
        } catch (Throwable e) {
            logger.warn("error evaluating chromosome " + c.toString(), e);
            c.setFitnessValue(0);
        }
    }

    private int singleTrial(Activator activator) {
        Board game;
        Pacman pacman = null;
        if (showGame) {
            pacman = new Pacman(true);
            game = pacman.b;
            pacman.stepFrame(true);
        } else {
            game = new Board();

            game.titleScreen = false;
            game.reset();
            game.currScore = 0;
            /* Send the game map to player and all ghosts */
            game.player.updateState(game.state);
            /* Don't let the player go in the ghost box*/
            game.player.state[9][7] = false;
            game.ghost1.updateState(game.state);
            game.ghost2.updateState(game.state);
            game.ghost3.updateState(game.state);
            game.ghost4.updateState(game.state);
        }
        int fitness = 0;

        // Run the pole-balancing simulation.
        int currentTimestep = 0;
        for (currentTimestep = 0; currentTimestep < maxTimesteps; currentTimestep++) {
            // Network activation values
            double[] networkInput = getNetworkInput(game);

            // Activate the network.
            double[] networkOutput = activator.next(networkInput);

            int maxI = -1;
            double maxV = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < 4; ++i) {
                if (networkOutput[i] > maxV) {
                    maxI = i;
                    maxV = networkOutput[i];
                }
            }
            switch (maxI) {
                case 0:
                    game.player.desiredDirection = 'L';
                    break;
                case 1:
                    game.player.desiredDirection = 'R';
                    break;
                case 2:
                    game.player.desiredDirection = 'U';
                    break;
                case 3:
                    game.player.desiredDirection = 'D';
                    break;
                default:
                    throw new RuntimeException("This shouldn't happen");
            }

            if (showGame) {
                pacman.stepFrame(false);
                game.repaint(0, 0, 600, 600);
                try {
                    Thread.sleep(60);
                } catch (InterruptedException ex) {
                    java.util.logging.Logger.getLogger(DoublePoleBalanceFitnessFunction.class.getName()).log(Level.SEVERE, null, ex);
                }
            } else {
                game.step();
            }
            fitness = game.currScore;

            //System.out.println(game.currScore);
            if (game.stopped || game.winScreen || game.overScreen || game.titleScreen) {
                break;
            }
        }
        
        System.out.println("DONE?");

        logger.debug("trial took " + currentTimestep + " steps");
        return fitness;
    }

    public double[] getNetworkInput(Board game) {
        double[] input = new double[9 * 9 * 2 + 1];
        int p = 0;
        if (game == null || game.state == null || game.pellets == null) {
            return input;
            //throw new RuntimeException("Game is not initialized");
        }

        for (int x = game.player.pelletX - 4; x <= game.player.pelletX + 4; ++x) {
            for (int y = game.player.pelletY - 4; y <= game.player.pelletY + 4; ++y) {
                if (x < 0 || y < 0 || x >= 20 || y >= 20) {
                    continue;
                }
                input[p++] = game.state[x][y] ? 1.0 : 0.0;
                input[p++] = game.pellets[x][y] ? 1.0 : 0.0;
            }
        }

        input[p] = 1.0;

        return input;
    }

    /**
     * @see org.jgap.BulkFitnessFunction#getMaxFitnessValue()
     */
    @Override
    public int getMaxFitnessValue() {
        return 50 * 173 * numTrials;
    }

    /**
     * enable GUI display of pole balancing
     */
    public void enableDisplay() {
        showGame = true;
    }
}
