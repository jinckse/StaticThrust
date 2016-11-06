/* speedtest.c */ 
#include <stdlib.h> 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareTask(Task1);

/* LEJOS OSEK hooks */
void user_1ms_isr_type2(void){}

/* Constants */
#define FIELD_M 10                              /*Number of rows in the field*/
#define FIELD_N 16                              /*Number of columns in the field*/
#define CELL_WIDTH 0.305
#define STEP 0.1525
#define NUM_OBSTACLES 14
#define MAX_DISTANCE 161
#define MAX_OBSTACLES 25                        /* maximum number of obstacles */
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  /*Used to round doubles to longs*/

#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4



/*
* PROTOTYPES
*/
//void Initialize(void);
//void ConvertLocations(void);
//void PlaceObject(void);
//int Manhattan(void);
//void PathFinder(void);

/*
 * GLOBAL VARS
 */

/*Struct to be used in the Map array*/
struct Map{
    long val;
    int available;
    int complete;
    int noTouch;
};



double startInitial[2] = {0.305*2, 0.305*9};        /* start location */
double goalInitial[2] = {3.658, 1.829};         /* goal location */
long startFinal[2];                              /* start location translated to block location */
long goalFinal[2] ;                              /* goal location translated to block location*/
long obstacleFinal[NUM_OBSTACLES][2];            /*holds block locations of obstacles*/
struct Map Map1[FIELD_M][FIELD_N];              /*Structure array to be used for the map*/

int route[20];				/*Used to hold final directions for the path*/


/* Sub functions */
int getRandom(int min, int max)
{
	return min + (int)((double)rand()*(max-min+1.0)/(1.0+RAND_MAX));
}

/* 
 * LEJOS OSEK V1.07 has an interrupt based LCD display feature thanks to LEJOS NXJ develpment team.
 * Therefore, LCD display performance is dramatically improved. Actually, LCD update is too
 * fast to see. The purpose of this speed test is to measure the performance of LEJOS OSEK under
 * a common program load compared to other programming languages. (The maximum LCD refresh rate
 * on hardware level is 60Hz, so faster refresh rate than 60Hz has no meaning in practical use.)
 * So the result could be seen only at the end of the test
 */
void disp(int row, char *str, int val)
{
#define DISPLAY_ON
#ifdef DISPLAY_ON 
	display_clear(0);
	display_goto_xy(0, row);
	display_string(str);
	display_int(val, 0);
	display_update();
#endif
}
/* Robot Control Functions */
void move(){
	int rev;
	int cnt = 0;
	nxt_motor_set_count(NXT_PORT_B, 0);
	
	do {
		rev = nxt_motor_get_count(NXT_PORT_B);
		disp(2, " REV: ", rev);
		
		if(rev < 425){
			nxt_motor_set_speed(NXT_PORT_B, 29, 1);
			nxt_motor_set_speed(NXT_PORT_C, 28, 1);
		} else
			cnt++;
	} while(cnt < 5);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	systick_wait_ms(500);
	
}

void moveHalf(){
	int rev;
	int cnt = 0;
	nxt_motor_set_count(NXT_PORT_B, 0);
	
	do {
		rev = nxt_motor_get_count(NXT_PORT_B);
		disp(2, " REV: ", rev);
		
		if(rev < 213){
			nxt_motor_set_speed(NXT_PORT_B, 25, 1);
			nxt_motor_set_speed(NXT_PORT_C, 25, 1);
		} else
			cnt++;
	} while(cnt < 5);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	systick_wait_ms(500);
	
}

void back(){
	int rev;
	int cnt = 0;
	nxt_motor_set_count(NXT_PORT_B, 0);
	
	do {
		rev = nxt_motor_get_count(NXT_PORT_B);
		disp(2, " REV: ", rev);
		
		if(rev < 450){
			nxt_motor_set_speed(NXT_PORT_B, -45, 1);
			nxt_motor_set_speed(NXT_PORT_C, -46, 1);
		} else
			cnt++;
	} while(cnt < 5);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	systick_wait_ms(500);
	
}

void turnLeft(){
	int rev;
	int cnt = 0;
	nxt_motor_set_count(NXT_PORT_B, 0);

	do{
		rev = nxt_motor_get_count(NXT_PORT_B);
		disp(2, " REV: ", rev);
	
		if(rev < 110){
			//To turn left, C is negative & B is positive.
			nxt_motor_set_speed(NXT_PORT_B, 25, 1);
			nxt_motor_set_speed(NXT_PORT_C, -26, 1);
		} else
			cnt++;
		
	} while(cnt < 5);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	systick_wait_ms(500);
}

void turnRight(){
	int rev;
	int cnt = 0;
	nxt_motor_set_count(NXT_PORT_C, 0);

	do{
		rev = nxt_motor_get_count(NXT_PORT_C);
		disp(2, " REV: ", rev);
	
		if(rev < 110){
			//To turn right, C is positive & B is negative.
			nxt_motor_set_speed(NXT_PORT_B, -25, 1);
			nxt_motor_set_speed(NXT_PORT_C, 30, 1);
		} else
			cnt++;
		
	} while(cnt < 5);
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	systick_wait_ms(500);
}

void hardPath(){
	move();
	//turnRight();
	//turnLeft();
	move();
	//turnRight();
	//turnLeft();
	move();
	//turnRight();
	//turnLeft();
	move();
	//turnRight();
	//turnLeft();
	move();
	
}

void Initialize(){

	int i,j;

	for (i=0; i< FIELD_M; i++){
		for (j=0; j<FIELD_N; j++){
			Map1[i][j].val = 0;
			Map1[i][j].available = 1;
			Map1[i][j].complete = 0;
			Map1[i][j].noTouch = 0;
		}
	}
}

void ConvertLocations(void){
	int i;


	/* obstacle locations */
	double obstacleInitial[NUM_OBSTACLES][2] =
	{{0.305*3, 0.305*5},{0.305*3, 0.305*6},{0.305*3, 0.305*7},{0.305*3, 0.305*8},
	{0.305*3, 0.305*9},{ 0.305*6, 0.305},{0.305*6, 0.305*2},{0.305*6, 0.305*3},
	{0.305*6, 0.305*4},{0.305*6, 0.305*5},{0.305*9, 0.305*6},{0.305*9, 0.305*7},
	{0.305*9, 0.305*8},{0.305*9, 0.305*9}};

    /*Changing values for obstacles*/
	for (i=0; i<NUM_OBSTACLES; i++){
        	//obstacleAns1 = obstacleInitial[i][0]/CELL_WIDTH;
        	obstacleFinal[i][1] = round(obstacleInitial[i][0]/CELL_WIDTH)-1;
        	obstacleFinal[i][0] = 10-round(obstacleInitial[i][1]/CELL_WIDTH);
    	
    	}
	
    /*Changing values for start*/
	startFinal[1] = round(startInitial[0]/CELL_WIDTH)-1;
    	startFinal[0] = 10-round(startInitial[1]/CELL_WIDTH)-1;
    /*Changing values for goal*/
    	goalFinal[1] = round(goalInitial[0]/CELL_WIDTH)-1;
    	goalFinal[0] = 10-round(goalInitial[1]/CELL_WIDTH)-1;
}

/*
 *Adding Obstacles
 */
void PlaceObject(void){
    int i;
    /*
    *Adding Obstacles to map array
    */
    for (i = 0; i < NUM_OBSTACLES; i++) {
        /*Filling Cell*/
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]].val = -2;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]].available = 0;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]].complete = 0;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]].noTouch = 1;
        /*Filling cell above */
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]].val = -2;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]].available = 0;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]].complete = 0;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]].noTouch = 1;
        /*Filling cell to right*/
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]+1].val = -2;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]+1].available = 0;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]+1].complete = 0;
        Map1[obstacleFinal[i][0]][obstacleFinal[i][1]+1].noTouch = 1;
        /*Filling cell above and to right*/
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]+1].val = -2;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]+1].available = 0;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]+1].complete = 0;
        Map1[obstacleFinal[i][0]-1][obstacleFinal[i][1]+1].noTouch = 1;
	//printf("Inside PlaceObject\n");
    }

    /*Adding Start location to map array*/
    Map1[startFinal[0]][startFinal[1]].val = -2;
    Map1[startFinal[0]][startFinal[1]].available = 0;
    Map1[startFinal[0]][startFinal[1]].complete = 0;
    Map1[startFinal[0]][startFinal[1]].noTouch = 1;

    //printf("Added Start Location\n");	
    /*
    *Adding goal to the array
    */
    Map1[goalFinal[0]][goalFinal[1]].val = -1;
    Map1[goalFinal[0]][goalFinal[1]].available = 0;
    Map1[goalFinal[0]][goalFinal[1]].complete = 1;
    Map1[goalFinal[0]][goalFinal[1]].noTouch = 1;
    /*Filling cell above*/
    Map1[goalFinal[0]+1][goalFinal[1]].val = -1;
    Map1[goalFinal[0]+1][goalFinal[1]].available = 0;
    Map1[goalFinal[0]+1][goalFinal[1]].complete = 1;
    Map1[goalFinal[0]+1][goalFinal[1]].noTouch = 1;
    /*Filling cell to right*/
    Map1[goalFinal[0]][goalFinal[1]+1].val = -1;
    Map1[goalFinal[0]][goalFinal[1]+1].available = 0;
    Map1[goalFinal[0]][goalFinal[1]+1].complete = 1;
    Map1[goalFinal[0]][goalFinal[1]+1].noTouch = 1;
    /*Filing cell above and to the right*/
    Map1[goalFinal[0]+1][goalFinal[1]+1].val = -1;
    Map1[goalFinal[0]+1][goalFinal[1]+1].available = 0;
    Map1[goalFinal[0]+1][goalFinal[1]+1].complete = 1;
    Map1[goalFinal[0]+1][goalFinal[1]+1].noTouch = 1;
    //printf("Added goal\n");
}

/*

 * Function Name: Manhattan
 * Description: Implement cell decomposition on game field
 * Parameters: None
 * Returns: int ret
 */
int Manhattan(void) {
    int i, j,k,l;
    int current;
    int counter = 1;
    int exit = 0;
    //printf("Starting Manhattan \n");

    while (exit == 0){

        /* Begin decomposition */
	    /*Filling Values*/
        for (i = 0; i < FIELD_M; i++) {
            for (j = 0; j < FIELD_N; j++) {

                /* Only map cells that haven't been completed */
                if(Map1[i][j].available == 1){

                    Map1[i][j].val = counter;
		}		
	    }
	}
	/*adjusting completed cells (designates value can not be changed)*/
        for (i = 0; i < FIELD_M; i++) {
            for (j = 0; j < FIELD_N; j++) {
                    if (Map1[i][j].complete == 1 && Map1[i][j].available == 0 && Map1[i][j].val != -2) {
			    Map1[i][j].noTouch = 1;
		    	    /*Completing surrounding cells*/

			    if (i+1 <10){
                        	Map1[i+1][j].complete = 1;	/*Down*/

			    }
			    if (i-1 >=0){
				Map1[i-1][j].complete = 1;	/*Up*/

			    }
			    if (j+1 < 16){
		    		Map1[i][j+1].complete = 1;	/*Right*/

			    }
			    if(j-1 >= 0){
				Map1[i][j-1].complete = 1;	/*Left*/

			    }
		    }
	    }
	}
	/*Adjusting Available flag for completed cells (removes location from the next value update)*/
        for (i = 0; i < FIELD_M; i++) {
            for (j = 0; j < FIELD_N; j++) {
                    if (Map1[i][j].complete == 1 && Map1[i][j].noTouch == 0) {
			/*Completing surrounding cells*/

                        	Map1[i][j].available  = 0;
				Map1[i][j].noTouch = 1;
		    }
            }
        }

        counter += 1;

        if (Map1[0][0].complete == 1){
            exit = 1;
        }
    }

	for (i = 0; i < 10; i++) {
		for (j = 0; j < 10; j++) {
			//printf("%3d ", Map1[i][j]);
		}
		//printf("\n");
	}
    return 1;
}

/* 

 *	Starting here is the algorithm for finding a path
 *	through the Manhattan populated map for the robot
 *	to follow.
 * */

void PathFinder(){
	
	int count= 0;
	int dir,i;


	/*Struct used in path finding*/
	struct Placement{
		long val;
		long position[2];
	};

	/*Creating struct to hold current position and value*/
	struct	Placement Place;			
	Place.position[0] = startFinal[0];			
	Place.position[1] = startFinal[1];
	Place.val = 99;

	for (i=0; i<50; i++){
		route[i] = 0;
	}

	while(Place.val != -1){
		/*compare up and right*/
		if (Place.position[0]-1 >=0){
			if (Place.val > Map1[Place.position[0]-1][Place.position[1]].val &&  Map1[Place.position[0]-1][Place.position[1]].val != -2 ){
				dir = 1; 
				Place.val = Map1[Place.position[0]-1][Place.position[1]].val;
			}
		}	
		 if ( Place.position[1]+1 < 16 && Map1[Place.position[0]][Place.position[1]+1].val != -2){
			if(Place.val > Map1[Place.position[0]][Place.position[1]+1].val && Map1[Place.position[0]][Place.position[1]+1].val != -2){	
	 		 	dir = 2;
				Place.val = Map1[Place.position[0]][Place.position[1]+1].val;
			}
		}
		/*compare with down*/
		if(Place.position[0]+1 < 10 &&  Map1[Place.position[0]+1][Place.position[1]].val != -2){
			if (Place.val > Map1[Place.position[0]+1][Place.position[1]].val){
				dir = 3; 
				Place.val = Map1[Place.position[0]+1][Place.position[1]].val;
			}	
		}
		/*compare with left */
		if(Place.position[1]-1 >= 0 &&  Map1[Place.position[0]][Place.position[1]-1].val != -2){
			if (Place.val > Map1[Place.position[0]][Place.position[1]-1].val){
				dir = 4; 
				Place.val = Map1[Place.position[0]][Place.position[1]-1].val;
			}	
		}


		switch(dir){
		
			case 1:
				route[count] = 1;
				Place.position[0] -= 1;					/*move position North one space*/

				break;
			case 2:
				route[count] = 2;
				Place.position[1] += 1;					/*move position East one space*/

				break;
			case 3:
				route[count] = 3;
				Place.position[0] += 1;					/*move position South one space*/
				break;
			case 4:
				route[count] = 4;				
				Place.position[1] -= 1;					/*move position West one space*/
				break;
			default:
				break;
		}
		count ++;
		
	}
}

void softPath(){
	int i = 0;
	moveHalf();
	turnLeft();
	moveHalf();
	int curDirection = SOUTH;
	while(route[i] != -1){
		if(route[i] == NORTH){
			if(curDirection == NORTH){
				move();			
			}
			if(curDirection == EAST){
				turnLeft();
				move();			
			}
			if(curDirection == SOUTH){
				turnLeft();
				turnLeft();
				move();			
			}
			if(curDirection == WEST){
				turnRight();
				move();			
			}
			curDirection = NORTH;
		}
		if(route[i] == EAST){
			if(curDirection == NORTH){
				turnRight();
				move();			
			}
			if(curDirection == EAST){
				move();			
			}
			if(curDirection == SOUTH){
				turnLeft();
				move();			
			}
			if(curDirection == WEST){
				turnLeft();
				turnLeft();
				move();			
			}
			curDirection = EAST;
		}
		if(route[i] == SOUTH){
			if(curDirection == NORTH){
				turnLeft();
				turnLeft();
				move();			
			}
			if(curDirection == EAST){
				turnRight();
				move();			
			}
			if(curDirection == SOUTH){
				move();			
			}
			if(curDirection == WEST){
				turnLeft();
				move();			
			}
			curDirection = SOUTH;
		}
		if(route[i] == WEST){
			if(curDirection == NORTH){
				turnLeft();
				move();			
			}
			if(curDirection == EAST){
				turnLeft();
				turnLeft();
				move();			
			}
			if(curDirection == SOUTH){
				turnRight();
				move();			
			}
			if(curDirection == WEST){
				move();			
			}
			curDirection = WEST;
		}
		i++;	
	}
}


/* Task for speed test */
TASK(Task1)
{
	systick_wait_ms(1000);
	Initialize();
	ConvertLocations();
	PlaceObject();
	Manhattan();
	PathFinder();
	softPath();
	//hardPath();
	
	nxt_motor_set_speed(NXT_PORT_B, 0, 1); 			
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);

						
	nxt_motor_set_speed(NXT_PORT_A, 0, 1);					
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);
	
	TerminateTask();
}
