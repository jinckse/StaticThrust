/*
 * File: path.c
 * Description: Path planning function C module
 */

#include <stdio.h>

/*
 * CONSTANTS
 */
#define FIELD_M 10
#define FIELD_N 16
#define CELL_WIDTH 0.305
#define STEP 0.1525
#define MAX_DISTANCE 160

/* 
 * MACROS
 */

/*
 * STRUCTURES
 */

/* Cell state */
struct State {
	int pathMarker;
	int empty;
	int obst;
	int start;
	int goal;
};

/* Single cell */
struct Cell {
	double x;
	double y;
	struct State s;
	double value;
};

/*
 * GLOBAL VARS
 */
struct Cell g_field[FIELD_M][FIELD_N];
int g_obstW = 3;
int g_obstH = 2;
int cnt = 1;

/*
 * Function Name: Init
 * Description: Initialize system
 * Returns: int ret
 */
int Init() {
	int i,j;

	/* Initialize field */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			g_field[i][j].x = j;
			g_field[i][j].y = i;
			g_field[i][j].s.pathMarker = 0;
			g_field[i][j].s.empty = 1;
			g_field[i][j].s.obst = 0;
			g_field[i][j].s.start = 0;
			g_field[i][j].s.goal = 0;
			g_field[i][j].value = MAX_DISTANCE;
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
	int current;

	/* Find paths */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
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
			g_field[i + h][j + w].s.empty = 0;
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
			printf("%4.0f ", g_field[i][j].value);
		}
		printf("\n");
	}
}

/* Main routine */
int main() {
	Init();
	Print_Field();
	printf("\n");
	Gen_Obst(g_obstW, g_obstH);
	Print_Field();
	printf("\n");
	Manhattan();
	Print_Field();
}
