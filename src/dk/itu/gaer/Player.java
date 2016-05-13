package dk.itu.gaer;


import java.util.HashSet;
import java.util.Set;



/* This is the pacman object */
public class Player extends Mover {

    /* Direction is used in demoMode, currDirection and desiredDirection are used in non demoMode*/
    char direction;
    char currDirection;
    public char desiredDirection;

    /* Keeps track of pellets eaten to determine end of game */
    int pelletsEaten;

    /* Last location */
    int lastX;
    int lastY;

    /* Current location */
    public int x;
    public int y;

    /* Which pellet the pacman is on top of */
    public int pelletX;
    public int pelletY;

    /* teleport is true when travelling through the teleport tunnels*/
    boolean teleport;

    /* Stopped is set when the pacman is not moving or has been killed */
    boolean stopped = false;

    /* Constructor places pacman in initial location and orientation */
    public Player(int x, int y) {

        teleport = false;
        pelletsEaten = 0;
        pelletX = x / gridSize - 1;
        pelletY = y / gridSize - 1;
        this.lastX = x;
        this.lastY = y;
        this.x = x;
        this.y = y;
        currDirection = 'L';
        desiredDirection = 'L';
    }

    /* The move function moves the pacman for one frame in non demo mode */
    public void move() {
        int gridSize = 20;
        lastX = x;
        lastY = y;

        /* Try to turn in the direction input by the user */
 /*Can only turn if we're in center of a grid*/
        if (x % 20 == 0 && y % 20 == 0
                || /* Or if we're reversing*/ (desiredDirection == 'L' && currDirection == 'R')
                || (desiredDirection == 'R' && currDirection == 'L')
                || (desiredDirection == 'U' && currDirection == 'D')
                || (desiredDirection == 'D' && currDirection == 'U')) {
            switch (desiredDirection) {
                case 'L':
                    if (isValidDest(x - increment, y)) {
                        x -= increment;
                    }
                    break;
                case 'R':
                    if (isValidDest(x + gridSize, y)) {
                        x += increment;
                    }
                    break;
                case 'U':
                    if (isValidDest(x, y - increment)) {
                        y -= increment;
                    }
                    break;
                case 'D':
                    if (isValidDest(x, y + gridSize)) {
                        y += increment;
                    }
                    break;
            }
        }
        /* If we haven't moved, then move in the direction the pacman was headed anyway */
        if (lastX == x && lastY == y) {
            switch (currDirection) {
                case 'L':
                    if (isValidDest(x - increment, y)) {
                        x -= increment;
                    } else if (y == 9 * gridSize && x < 2 * gridSize) {
                        x = max - gridSize * 1;
                        teleport = true;
                    }
                    break;
                case 'R':
                    if (isValidDest(x + gridSize, y)) {
                        x += increment;
                    } else if (y == 9 * gridSize && x > max - gridSize * 2) {
                        x = 1 * gridSize;
                        teleport = true;
                    }
                    break;
                case 'U':
                    if (isValidDest(x, y - increment)) {
                        y -= increment;
                    }
                    break;
                case 'D':
                    if (isValidDest(x, y + gridSize)) {
                        y += increment;
                    }
                    break;
            }
        } /* If we did change direction, update currDirection to reflect that */ else {
            currDirection = desiredDirection;
        }

        /* If we didn't move at all, set the stopped flag */
        if (lastX == x && lastY == y) {
            stopped = true;
        } /* Otherwise, clear the stopped flag and increment the frameCount for animation purposes*/ else {
            stopped = false;
            frameCount++;
        }
    }

    /* Update what pellet the pacman is on top of */
    public void updatePellet() {
        if (x % gridSize == 0 && y % gridSize == 0) {
            pelletX = x / gridSize - 1;
            pelletY = y / gridSize - 1;
        }
    }
}