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
#define MAX_DISTANCE 160

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
int g_obstW = 3;
int g_obstH = 2;
int cnt = 1;
int debug = 1;

/*
 * Function Name: Init
 * Description: Initialize system
 * Returns: int ret
 */
int Init() {
	int i,j,k;

	/* Initialize field */
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
		}
	}
	
	/* Set start and goal locations */
	for (i=0; i < FIELD_M; i++) {
		for(j = 0; j < FIELD_N; j++) {
			if ( (g_field[i][j].x == Meters_To_Feet(g_start[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(g_start[1])) ) {
				g_field[i][j].s.start = 1;
				g_field[i][j].s.processed = 1;
				g_field[i][j].value = 111;
			}
			else if ( (g_field[i][j].x == Meters_To_Feet(g_goal[0]) )
				&& (g_field[i][j].y == Meters_To_Feet(g_goal[1])) ) {
				g_field[i][j].s.goal = 1;
				g_field[i][j].s.processed = 1;
				g_field[i][j].value = 999;
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
					g_field[i][j].s.processed = 1;
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
				g_field[i][j].value = -1;
			}
		}
	}

	return 1;
}

/*
 * Function Name: Manhattan
 * Description: Implement Manhattan distancing on game field
 * Parameters: None
 * Returns: int ret
 */
int Manhattan(void) {
	int i, j;

	/* Find paths */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			/* Only populate non obstacles and start / goal points */
			if ( !Is_Proc_Adjacent(g_field, i, j) ) {
				g_field[i][j].value = cnt;		
				printf("Got Here...\n");
			}
		}
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
 * Returns: int r
 */
int Meters_To_Feet(double m) {
	int r = 0;
	double iptr;
	double in;
	
	m = m / CELL_WIDTH;

	/* Store integral part of m */
	in = modf(m, &iptr);
	
	if (in >= 0.5)
		m++;

	r = (int) m;
	
	if(debug) {
		printf("in: %f\n",in);
		printf("r: %d\n", r);
	}

	return r;
}

/*
 * Function Name: Is_Proc_Adjacent
 * Description: Determine if a processed point is nearby
 * Parameters: struct Point p, int x, int y
 * Returns: int r
 */
int Is_Proc_Adjacent(struct Point p[FIELD_N][FIELD_M], int x, int y) {
	/* Check all directions */
		return 0;
}

/* Main routine */
int main() {
	Init();
	Print_Field();
	printf("\n");
	Manhattan();
	Print_Field();
	printf("\n");
}
