/*
 * File: path.c
 * Description: Path planning function C module
 */

#include <stdio.h>
#include <math.h>
#include "global.h"

/*
 * CONSTANTS
 */
#define FIELD_M 11
#define FIELD_N 17
#define CELL_WIDTH 0.305
#define STEP 0.1525
#define MAX_DISTANCE 16

#define NORMAL 1
#define CORNER 2
#define EDGE 3

#define NORTH 10
#define SOUTH 20
#define EAST  30
#define WEST 40

#define MOVE 4
#define TURN_L 5
#define TURN_R 6

/*
 * MACROS
 */

/*
 * STRUCTURES
 */

/* Point state */
struct State {
	int pathMarker;
	int processed;
	int next;
	int obst;
	int start;
	int goal;
};

/* Single point */
struct Point {
	double x;
	double y;
	struct State s;
	double value;
	int type;
	struct Point* n_up;
	struct Point* n_down;
	struct Point* n_left;
	struct Point* n_right;
	int move;
};

/* Instructions */
struct Instructions {
    int length;
    int *instruction_list;
};

/*
 * GLOBAL VARS
 */
struct Point g_field[FIELD_M][FIELD_N];
struct Point *g_goalPoint, *g_startPoint;
int g_debug = 1;
struct Point g_path[FIELD_M * FIELD_N];

/*
 * Function Name: Init
 * Description: Initialize grid
 * Returns: int ret
 */
int Init() {
	int i,j,k;

	/* Initialize points */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			g_field[i][j].x = j;
			g_field[i][j].y = FIELD_M - (i + 1);
			g_field[i][j].s.pathMarker = 0;
			g_field[i][j].s.processed = 0;
			g_field[i][j].s.next = 0;
			g_field[i][j].s.obst = 0;
			g_field[i][j].s.start = 0;
			g_field[i][j].s.goal = 0;
			g_field[i][j].value = -1;

			/* Conditionally set point types */

			/* Set corners */
			if (
					( (j == 0) && (i == 0) ) ||
					( (j == FIELD_N - 1) && (i == 0) ) ||
					( (j == 0) && (i == FIELD_M - 1) ) ||
					( (j == FIELD_N - 1) && (i == FIELD_M - 1) )
				) {
				g_field[i][j].type = CORNER;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
				g_field[i][j].value = -88;
			}

			/* Set edges */
			else if (
					( ( (i == 0) || (j == 0) ) && (g_field[i][j].type != CORNER) ) ||
					( ( (i == FIELD_M-1) || (j == FIELD_N-1) ) && (g_field[i][j].type != CORNER) )
				) {
				g_field[i][j].type = EDGE;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
				g_field[i][j].value = -77;
			}
			else {
				g_field[i][j].type = NORMAL;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
			}
		}
	}

	/* Set start and goal locations */
	for (i = 0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			if ( (g_field[i][j].x == Meters_To_Feet(start[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(start[1])) ) {
				g_field[i][j].s.start = 1;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
				g_field[i][j].value = (FIELD_M * FIELD_N);;
				g_startPoint = &g_field[i][j];
			}
			else if ( (g_field[i][j].x == Meters_To_Feet(goal[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(goal[1])) ) {
				g_field[i][j].s.goal = 1;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
				g_field[i][j].value = 0;
				g_goalPoint = &g_field[i][j];
			}
		}
	}

	/* Set obstacle locations*/
	for (i = 0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			for (k = 0; k < MAX_OBSTACLES; k++) {
				if ( (g_field[i][j].x == Meters_To_Feet(obstacle[k][0]) )
					&& (g_field[i][j].y == Meters_To_Feet(obstacle[k][1]) ) ) {
					g_field[i][j].s.obst = 1;
					g_field[i][j].s.processed = 0;
					g_field[i][j].s.next = 0;
					g_field[i][j].value = -66;
				}
			}
		}
	}

	/* Set path locations*/
	for (i = 0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			if ( (!g_field[i][j].s.start) && (!g_field[i][j].s.goal) &&
					(!g_field[i][j].s.obst) && (g_field[i][j].type != EDGE) &&
					(g_field[i][j].type != CORNER) ) {
				g_field[i][j].s.pathMarker = 1;
				g_field[i][j].s.processed = 0;
				g_field[i][j].s.next = 0;
			}
		}
	}

	/* Assign neighbors */
	for (i = 0; i < FIELD_M; i++) {
		for (j= 0; j < FIELD_N; j++) {
			if ( (g_field[i][j].type != CORNER) && g_field[i][j].type != EDGE) {

				/* Connect adjacent neighbors */
				g_field[i][j].n_up = &g_field[i - 1][j];
				g_field[i][j].n_down = &g_field[i + 1][j];
				g_field[i][j].n_left = &g_field[i][j - 1];
				g_field[i][j].n_right = &g_field[i][j + 1];
			}
		}
	}

	return 1;
}

/*
 * Function Name: Manhattan
 * Description: Implement Manhattan distancing on game field
 * Parameters: int block
 * Returns: int ret
 */
int Manhattan(int block) {
	int i, j;

	/* Find paths */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			if (
					g_field[i][j].type != CORNER &&
					g_field[i][j].type != EDGE &&
					!g_field[i][j].s.start &&
					!g_field[i][j].s.goal &&
					!g_field[i][j].s.obst &&
					!g_field[i][j].s.processed
				) {

				/* In first pass check for goal */
				if(block == 1) {
					if (
						g_field[i][j].n_up -> s.goal ||
						g_field[i][j].n_down -> s.goal ||
						g_field[i][j].n_left -> s.goal ||
						g_field[i][j].n_right -> s.goal
					) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;

						/* Lock adjacent points */
						g_field[i][j].n_up -> s.next = 1;
						g_field[i][j].n_down -> s.next = 1;
						g_field[i][j].n_left -> s.next = 1;
						g_field[i][j].n_right -> s.next = 1;
					}
				}
				else {
					if (g_field[i][j].s.next) {

						/* Update current path point */
						g_field[i][j].s.next = 0;
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;
						}
					}
				}
			}
		}

	/* One more check to update processed point neighbors */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			if (g_field[i][j].value == block) {

				/* Lock adjacent points */
				g_field[i][j].n_up -> s.next = 1;
				g_field[i][j].n_down -> s.next = 1;
				g_field[i][j].n_left -> s.next = 1;
				g_field[i][j].n_right -> s.next = 1;

			}
		}
	}

	/* Recursive call */
	if (block < MAX_DISTANCE) {
		Manhattan(++block);
	}

	return 1;
}

/*
 * Function Name: Print_Field
 * Description: Show field
 * Parameters: none
 * Returns: none
 */
void Print_Field(void) {
	int i,j;

	printf("VALUES:\n");
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			if(g_debug){
				printf("%3.0f ", g_field[i][j].value, i, j);
			}
		}
		printf("\n");
		printf("\n");
	}
}

/*
 * Function Name: Meters_To_feet
 * Description: Convert meters to feet and return whole number
 * Parameters: double m
 * Returns: int ret
 */
int Meters_To_Feet(double m) {
	int ret = 0;
	double iptr;
	double in;

	m = m / CELL_WIDTH;

	/* Store integral part of m */
	in = modf(m, &iptr);

	if (in >= 0.5)
		m++;

	ret = (int) m;

	return ret;
}

/*
 * Function Name: Find_Path
 * Description: Find a path to the goal and return it
 * Parameters: None
 * Returns: int r
 */
int Find_Path(void) {
	int i = 0;
	int cnt = 0;
	struct Point current = *g_goalPoint;

	/* Set up Manhattan mapped grid */
	Init();
	Manhattan(1);

	/* Start searching from goal */
	do {

		/* Not near an edge */
		if (current.x > 2 && current.y > 2) {

			/* Look two points ahead to determine path */
			if (current.n_up -> value > current.value &&
				current.n_up -> n_up -> value > current.value) {
				g_path[i] = current;
				current = *current.n_up;
				g_path[i].move = NORTH;
			}
			else if (current.n_down -> value > current.value &&
				current.n_down -> n_down -> value > current.value) {
				g_path[i] = current;
				current = *current.n_down;
				g_path[i].move = SOUTH;
			}
			else if (current.n_left -> value > current.value &&
				current.n_left -> n_left -> value > current.value) {
				g_path[i] = current;
				current = *current.n_left;
				g_path[i].move = WEST;
			}
			else if (current.n_right -> value > current.value &&
				current.n_right -> n_right -> value > current.value) {
				g_path[i] = current;
				current = *current.n_right;
				g_path[i].move = EAST;
			}

			/* Track number of points in path */
			cnt++;
		}
		else {

			/* Look one point ahead to determine path */
			if (current.n_up -> value > current.value) {
				g_path[i] = current;
				current = *current.n_up;
				g_path[i].move = NORTH;
			}
			else if (current.n_down -> value > current.value) {
				g_path[i] = current;
				current = *current.n_down;
				g_path[i].move = SOUTH;
			}
			else if (current.n_left -> value > current.value) {
				g_path[i] = current;
				current = *current.n_left;
				g_path[i].move = WEST;
			}
			else if (current.n_right -> value > current.value) {
				g_path[i] = current;
				current = *current.n_right;
				g_path[i].move = EAST;
			}

			/* Track number of points in path */
			cnt++;
		}
		g_path[i].s.processed = 1;
		i++;
	}
	while(current.value != g_startPoint -> value);

	/* Save start position */
	g_path[i] = current;
	cnt++;

	if (g_debug) {
		printf("PATH: \n");
		for (i = 0; i < cnt; i++) {
				printf("(%2.0f, %2.0f): %2.0f -> %2d\n",
					g_path[i].x, g_path[i].y, g_path[i].value, g_path[i].move);
		}
		printf("cnt = %d\n", cnt);
	}

	return cnt;
}

/*
 * Function Name: Path_To_Robot
 * Description: Given an array of points, convert it to an array of instructions.
 * Parameters: path_length is the length of the g_path array.
 * Returns: int[] instruction_list
 * Note: Iterates g_path in reverse, and direction of next.move is always the opposite direction that the robot wants to go.
 *       EX. (1,4) is the start, and (1,5) is next, but next.move = SOUTH even though robot wants to go NORTH.
 *       However, cur_direction is always the correct direction the robot is actually pointing.
 */
struct Instructions Path_To_Robot(int path_length){
    int i, cur_direction;
    int instruction_list[path_length * 3]; //worst case scenario every coordinate change takes 3 commands.
    int j = 0;
    printf("path_length is: %d\n", path_length);
    for(i = path_length - 1; i > 0; i = i - 1){
        printf("i: %d (%2.0f, %2.0f): %2.0f -> %2d\n",
					i,g_path[i].x, g_path[i].y, g_path[i].value, g_path[i].move);
        struct Point cur = g_path[i];
        struct Point next = g_path[i - 1];
        if(cur.move == 0){ /*ASSUME STARTING EAST*/
            //assume bot starts facing east
            cur_direction = EAST;
            if(next.move == NORTH){
                //turn right
                instruction_list[j] = TURN_R;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;

                cur_direction = SOUTH;
            }
            else if(next.move == SOUTH){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
                cur_direction = NORTH;
            }
            else if(next.move == EAST){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
                //curDirection = WEST
                cur_direction = WEST;
            }
            else if(next.move == WEST){
                //move
                instruction_list[j] = MOVE;
                j++;
                //curDirection = EAST
                cur_direction = EAST;
            }
        }
        else if(next.move == NORTH){ /* NORTH IS SOUTH*/
                if(cur_direction == NORTH){
                    //turn left
                    instruction_list[j] = TURN_L;
                    j++;
                    //turn left
                    instruction_list[j] = TURN_L;
                    j++;
                    //move
                    instruction_list[j] = MOVE;
                    j++;


                }
                else if(cur_direction == SOUTH){
                    //move
                    instruction_list[j] = MOVE;
                    j++;
                }
                else if(cur_direction == EAST){
                    //turn right
                    instruction_list[j] = TURN_R;
                    j++;
                    //move
                    instruction_list[j] = MOVE;
                    j++;
                }
                else if(cur_direction == WEST){
                    //turn left
                    instruction_list[j] = TURN_L;
                    j++;
                    //move
                    instruction_list[j] = MOVE;
                    j++;
                }
            cur_direction = SOUTH;
        }
        else if(next.move == SOUTH){ /*SOUTH IS NORTH*/
            if(cur_direction == NORTH){
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == SOUTH){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == EAST){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == WEST){
                //turn right
                instruction_list[j] = TURN_R;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            cur_direction = NORTH;
        }
        else if(next.move == EAST){ /*EAST IS WEST*/
            if(cur_direction == NORTH){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == SOUTH){
                //turn right
                instruction_list[j] = TURN_R;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == EAST){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == WEST){
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            cur_direction = WEST;
        }
        else if(next.move == WEST){ /*WEST IS EAST*/
            if(cur_direction == NORTH){
                //turn right
                instruction_list[j] = TURN_R;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == SOUTH){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == EAST){
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            else if(cur_direction == WEST){
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //turn left
                instruction_list[j] = TURN_L;
                j++;
                //move
                instruction_list[j] = MOVE;
                j++;
            }
            cur_direction = EAST;
        }
    }


    struct Instructions robot_path;
    robot_path.instruction_list = instruction_list;
    robot_path.length = j;

    return robot_path;
}

/*
* Function Name: Execute_Path
* Description: Read through the instruction_list array and calls the actual robot control functions
* Parameters: instruction_list array
* Returns: None*/
void Execute_Path(struct Instructions path){
    int i;
    for(i = 0; i < path.length; i++){
        if(path.instruction_list[i] == TURN_L)
            printf("Turn Left\n");
        else if(path.instruction_list[i] == TURN_R)
            printf("Turn Right\n");
        else if(path.instruction_list[i] == MOVE)
            printf("Move\n");
    }
}
/* Main routine */
int main() {
    int path_length;

	Init();
	Manhattan(1);
    path_length = Find_Path();
	Print_Field();

	/*Convert path and execute*/
	struct Instructions path;
	path = Path_To_Robot(path_length);
	Execute_Path(path);
}
