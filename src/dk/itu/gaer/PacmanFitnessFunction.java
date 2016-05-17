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
package dk.itu.gaer;

import java.util.Iterator;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgap.BulkFitnessFunction;
import org.jgap.Chromosome;

import com.anji.imaging.IdentifyImageFitnessFunction;
import com.anji.integration.Activator;
import com.anji.integration.ActivatorTranscriber;
import com.anji.util.Configurable;
import com.anji.util.Properties;
import com.anji.util.Randomizer;

import java.util.HashSet;
import java.util.logging.Level;

/**
 * This code is a port from Colin Green's SharpNEAT pole balancing code, which
 * in turn is a port from Ken Stanley's NEAT code.
 *
 * @author Derek James
 */
public class PacmanFitnessFunction implements BulkFitnessFunction, Configurable {

    private final static String TIMESTEPS_KEY = "pacman.timesteps";
    private final static String NUM_TRIALS_KEY = "pacman.trials";

    private final static int DEFAULT_TIMESTEPS = 10000;
    private int maxTimesteps = DEFAULT_TIMESTEPS;

    private final static int DEFAULT_NUM_TRIALS = 10;
    private int numTrials = DEFAULT_NUM_TRIALS;

    private final static Logger logger = Logger.getLogger(PacmanFitnessFunction.class);

    private ActivatorTranscriber factory;

    private Random rand;

    private boolean showGame = false;

    /**
     * @throws java.lang.Exception
     * @see com.anji.util.Configurable#init(com.anji.util.Properties)
     */
    @Override
    public void init(Properties props) throws Exception {
        try {
            factory = (ActivatorTranscriber) props.singletonObjectProperty(ActivatorTranscriber.class);
            maxTimesteps = props.getIntProperty(TIMESTEPS_KEY, DEFAULT_TIMESTEPS);
            numTrials = props.getIntProperty(NUM_TRIALS_KEY, DEFAULT_NUM_TRIALS);
            Randomizer randomizer = (Randomizer) props.singletonObjectProperty(Randomizer.class);
            rand = randomizer.getRand();
        } catch (Exception e) {
            throw new IllegalArgumentException("invalid properties: " + e.getClass().toString() + ": " + e.getMessage());
        }
    }

    /**
     * @param genotypes
     * @see org.jgap.BulkFitnessFunction#evaluate(java.util.List)
     * @see IdentifyImageFitnessFunction#evaluate(Chromosome)
     */
    @Override
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
        Pacman pacman = new Pacman(true, showGame);
        HashSet<String> history = new HashSet<>();
        int fitness = 0;
        int stuckCounter = 100;

        // Run the pole-balancing simulation.
        int currentTimestep;
        for (currentTimestep = 0; currentTimestep < maxTimesteps; currentTimestep++) {
            // Network activation values
            double[] networkInput = getNetworkInput(pacman.b);

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
                    pacman.b.player.desiredDirection = 'L';
                    break;
                case 1:
                    pacman.b.player.desiredDirection = 'U';
                    break;
                case 2:
                    pacman.b.player.desiredDirection = 'R';
                    break;
                case 3:
                    pacman.b.player.desiredDirection = 'D';
                    break;
                default:
                    throw new RuntimeException("This shouldn't happen");
            }

            pacman.stepFrame(false);
            if (showGame) {
                pacman.b.repaint(0, 0, 600, 600);
                try {
                    Thread.sleep(60);
                } catch (InterruptedException ex) {
                    java.util.logging.Logger.getLogger(PacmanFitnessFunction.class.getName()).log(Level.SEVERE, null, ex);
                }
            } else {
                pacman.b.paint(null);
            }
            fitness = pacman.b.currScore;

            String pos = pacman.b.player.pelletX + " - " + pacman.b.player.pelletY;
            if (history.contains(pos)) {
                stuckCounter--;
            } else {
                stuckCounter = 100;
                history.add(pos);
            }

            //System.out.println(game.currScore);
            if (pacman.b.stopped || pacman.b.winScreen || pacman.b.overScreen || pacman.b.titleScreen || stuckCounter <= 0) {
                break;
            }
        }

        pacman.stop();
        pacman.destroy();

        logger.debug("trial took " + currentTimestep + " steps");
        return fitness;
    }

    public double[] getNetworkInput(Board game) {
        double[] input = new double[5 * 5 * 3 + 1];
        int p = 0;
        if (game == null || game.state == null || game.pellets == null) {
            return input;
            //throw new RuntimeException("Game is not initialized");
        }

        for (int x = game.player.pelletX - 1; x <= game.player.pelletX + 1; ++x) {
            for (int y = game.player.pelletY - 1; y <= game.player.pelletY + 1; ++y) {
                if (x < 0 || y < 0 || x >= 20 || y >= 20) {
                    continue;
                }
                input[p++] = game.state[x][y] ? 1.0 : 0.0;
                input[p++] = game.pellets[x][y] ? 1.0 : 0.0;
                if((x == game.ghost1.pelletX && y == game.ghost1.pelletY)
                    ||(x == game.ghost2.pelletX && y == game.ghost2.pelletY)
                    ||(x == game.ghost3.pelletX && y == game.ghost3.pelletY)
                    ||(x == game.ghost4.pelletX && y == game.ghost4.pelletY)){
                    input[p] = 1.0;
                }
                p++;
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
