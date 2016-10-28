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

/*
 * GLOBAL VARS
 */
double g_field[FIELD_M][FIELD_N];
int g_obstW = 3;
int g_obstH = 2;

/*
 * Function Name: Init
 * Description: Initialize system
 * Returns: int ret
 */
int Init() {
	int i,j;

	/* Populate field */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			g_field[i][j] = MAX_DISTANCE;
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

	/* Determine where we are */
	int start[2] = {0, 0};

	/* Begin distancing */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			current = g_field[i][j];
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
			g_field[i + h][j + w] = 0;
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
			printf("%4.0f ", g_field[i][j]);
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
