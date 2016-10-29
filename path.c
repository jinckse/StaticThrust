/*
 * File: path.c
 * Description: Path planning function C module
 * TODO: Implement neighbor assignment for graph algorithm after general initializing
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
};

/*
 * GLOBAL VARS
 */
struct Point g_field[FIELD_M][FIELD_N];
struct Point g_goalPoint, g_startPoint;;
int g_obstW = 3;
int g_obstH = 2;
int cnt = 1;
int debug = 1;

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
				g_field[i][j].value = 888;
			}

			/* Set edges */
			else if ( 
					( ( (i == 0) || (j == 0) ) && (g_field[i][j].type != CORNER) ) ||
					( ( (i == FIELD_M-1) || (j == FIELD_N-1) ) && (g_field[i][j].type != CORNER) )
				) {
				g_field[i][j].type = EDGE;
				g_field[i][j].s.processed = 0;
				g_field[i][j].value = 777;
			}
			else {
				g_field[i][j].type = NORMAL;
				g_field[i][j].s.processed = 0;
			}
		}
	}

	/* Set start and goal locations */
	for (i=0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			if ( (g_field[i][j].x == Meters_To_Feet(g_start[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(g_start[1])) ) {
				g_field[i][j].s.start = 1;
				g_field[i][j].s.processed = 0;
				g_field[i][j].value = 111;
				g_startPoint = g_field[i][j];
			}
			else if ( (g_field[i][j].x == Meters_To_Feet(g_goal[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(g_goal[1])) ) {
				g_field[i][j].s.goal = 1;
				g_field[i][j].s.processed = 0;
				g_field[i][j].value = 999;
				g_goalPoint = g_field[i][j];
			}
		}
	}

	/* Set obstacle locations*/
	for (i = 0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			for (k = 0; k < MAX_OBSTACLES; k++) {
				if ( (g_field[i][j].x == Meters_To_Feet(g_obstacle[k][0]) )
					&& (g_field[i][j].y == Meters_To_Feet(g_obstacle[k][1]) ) ) {
					g_field[i][j].s.obst = 1;
					g_field[i][j].s.processed = 0;
					g_field[i][j].value = 666;
				}
			}
		}
	}

	/* Set path locations*/
	for (i = 0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			if ( (!g_field[i][j].s.start) && (!g_field[i][j].s.goal)
				&& (!g_field[i][j].s.obst) ) {
				g_field[i][j].s.pathMarker = 1;	
				g_field[i][j].s.processed = 0;
			}
		}
	}

	/* Assign neigbors */
	for (i = 0; i < FIELD_M; i++) {
		for (j= 0; j < FIELD_N; j++) {
			if ( (g_field[i][j].type != CORNER) && g_field[i][j].type != EDGE) {
				/* Connect adjacent neighbors */
				g_field[i][j].n_up = &g_field[i][j - 1];
				g_field[i][j].n_down = &g_field[i + 1][j];
				g_field[i][j].n_left = &g_field[i - 1][j];
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
					)
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
				}

				/* In remaining passes check for processed points and start */
				else {
					/* If current point is above and to the right of the goal */
					if ( ((g_field[i][j].y > g_goalPoint.y) && (g_field[i][j].x > g_goalPoint.x)) &&
						(g_field[i][j].n_down -> s.processed || g_field[i][j].n_left -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is above and to the left of the goal */
					else if ( ((g_field[i][j].y > g_goalPoint.y) && (g_field[i][j].x < g_goalPoint.x)) &&
						(g_field[i][j].n_down -> s.processed || g_field[i][j].n_right -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is below and to the right of the goal */
					else if ( ((g_field[i][j].y < g_goalPoint.y) && (g_field[i][j].x > g_goalPoint.x)) &&
						(g_field[i][j].n_up -> s.processed || g_field[i][j].n_left -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is below and to the left of the goal */
					else if ( ((g_field[i][j].y < g_goalPoint.y) && (g_field[i][j].x < g_goalPoint.x)) &&
						(g_field[i][j].n_up -> s.processed || g_field[i][j].n_right -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is above the goal */
					else if ( ((g_field[i][j].y > g_goalPoint.y) && (g_field[i][j].x == g_goalPoint.x)) &&
						(g_field[i][j].n_down -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is below the goal */
					else if ( ((g_field[i][j].y < g_goalPoint.y) && (g_field[i][j].x == g_goalPoint.x)) &&
						(g_field[i][j].n_up -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
						printf("Maybe?\n");
					}
					/* If current point is to the right of the goal */
					else if ( ((g_field[i][j].y == g_goalPoint.y) && (g_field[i][j].x > g_goalPoint.x)) &&
						(g_field[i][j].n_left -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					/* If current point is to the left of the goal */
					else if ( ((g_field[i][j].y == g_goalPoint.y) && (g_field[i][j].x < g_goalPoint.x)) &&
						(g_field[i][j].n_right -> s.processed)) {
						g_field[i][j].s.processed = 1;
						g_field[i][j].value = block;	
					}
					else {
						/* Should never get here */
						printf("Got there...\n");
					}
				}
			}	
		}
	}
	if (block < 2) {
		Manhattan(++block);
	}

	return 1;
}

/*
 * Function Name: Gen_Obst
 * Description: Generate an obstacle for the field
 * Parameters: w, h
 * Returns: int ret
 */
void Gen_Obst(int w, int h) {
	int i,j;

	/* Create obstacle */
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			g_field[i + h][j + w].value = 0;
			g_field[i + h][j + w].s.obst = 1;
		}
	}
}

/*
 * Function Name: Print_Field
 * Description: Show field
 * Parameters: none
 * Returns: none
 */
void Print_Field(void) {
	int i,j;

	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			//printf("%4.0f ", g_field[i][j].value);
			if(debug){
				printf("%3.0f ", g_field[i][j].value);
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
 * Function Name: Is_Proc_Adjacent
 * Description: Determine if a processed point is nearby
 * Parameters: struct Point p, int x, int y
 * Returns: int r
 */
int Is_Proc_Adjacent(struct Point p[FIELD_N][FIELD_M], int x, int y) {
		/* May not need this function anymore */
		return 0;
}

/* Main routine */
int main() {
	Init();
	Manhattan(1);
	printf("\n");
	Print_Field();
	printf("\n");
	if (debug) {
		printf("P(1,1) = %f\n", g_field[1][1].value);
		printf("Left = %f\n", g_field[1][1].n_left -> value);
		printf("Right = %f\n", g_field[1][1].n_right -> value);
		printf("Up = %f\n", g_field[1][1].n_up -> value);
		printf("Down = %f\n", g_field[1][1].n_down -> value);
		printf("goalPoint: (%f, %f)\n", g_goalPoint.x, g_goalPoint.y);
	}
}
