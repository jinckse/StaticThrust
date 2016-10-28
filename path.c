/*
 * File: path.c
 * Description: Path planning functions
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
			g_field[i][j] = MAX_DISTANCE + 1;
		}
	}

	return 1;
}

/*
 * Function Name: Manhattan
 * Description: Implement cell decomposition on game field
 * Parameters: None
 * Returns: int ret
 */
int Manhattan(void) {
	int i, j;
	int current;

	/* Determine where we are */
	int start[2] = {0, 0};

	/* Begin decomposition */
	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			current = g_field[i][j];

			/* Only map cells that haven't been updated */
			if (g_field[i][j] < 0) { 
				g_field[i][j] = 1;
				if ( (g_field[i+1][j] <= current) && (g_field[i+1][j] >  0) && (i+1 < FIELD_M) ) 
					g_field[i+1][j] = g_field[i][j] + 1;
				else if ( (g_field[i-1][j] <= current) && (g_field[i-1][j] > 0) && (i-1 >= 0) )
					g_field[i-1][j] = g_field[i][j] + 1;
				else if ( (g_field[i][j+1] <= current) && (g_field[i][j+1] > 0) && (j+1 < FIELD_N) )
					g_field[i][j+1] = g_field[i][j] + 1;
				else if ( (g_field[i][j-1] <= current) && (g_field[i][j-1] > 0) && (j-1 >= 0) )
					g_field[i][j-1] = g_field[i][j] + 1;
				else
					{ /* Should never get here */ }
			}
			else if(current > 0) { 
				if ( (g_field[i+1][j] < current) && (g_field[i+1][j] >  0) && (i+1 < FIELD_M) ) 
					g_field[i+1][j] = g_field[i][j] + 1;
				else if ( (g_field[i-1][j] < current) && (g_field[i-1][j] > 0) && (i-1 >= 0) )
					g_field[i-1][j] = g_field[i][j] + 1;
				else if ( (g_field[i][j+1] < current) && (g_field[i][j+1] > 0) && (j+1 < FIELD_N) )
					g_field[i][j+1] = g_field[i][j] + 1;
				else if ( (g_field[i][j-1] < current) && (g_field[i][j-1] > 0) && (j-1 >= 0) )
					g_field[i][j-1] = g_field[i][j] + 1;
				else
					{ /* Should never get here */ }
			}
			else
					{ /* Should never get here */ }
		}
	}
	return 1;
}

void Gen_Obst(int w, int h) {
	int i,j;

	/* Create obstacle */
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			g_field[i + h][j + w] = 0;
		}
	}
}

void Print_Field(void) {
	int i,j;

	for (i = 0; i < FIELD_M; i++) {
		for (j = 0; j < FIELD_N; j++) {
			printf("%4.0f ", g_field[i][j]);
		}
		printf("\n");
	}
}

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
