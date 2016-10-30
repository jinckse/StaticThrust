/*
 * File global.h
 * Description: global header file for Team 6 project 1
 */

/*
 * Contsants
 */
#define MAX_OBSTACLES 25

/*
 * Global Vars
 */
int numObstacles = 13;

double obstacle[MAX_OBSTACLES][2] = {
	{0.61, 2.743}, {0.915, 2.743}, {1.219, 2.743}, {1.829, 1.219},
	{1.829, 1.524}, {1.829, 1.829}, {1.829, 2.134}, {2.743, 0.305},
	{2.743, 0.61}, {2.743, 0.915}, {2.743, 2.743}, {3.048, 2.743},
	{3.353, 2.743},
	{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1},
	{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}
};

double start[2] = {0.305, 1.219}; 
double goal[2] = {3.658, 1.829};

/*
 * FUNCTION PROTOTYPES
 */
int Init(void);
int Manhattan(int block);
void Gen_Obst(int w, int h);
void Print_Field(void);
int Meters_To_Feet(double m);
int Find_Path(void);
