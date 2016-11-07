/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <stdio.h>
#include <string.h>
/*
 * CONSTANTS
 */
#define FIELD_M 10                              /*Number of rows in the field*/
#define FIELD_N 16                              /*Number of columns in the field*/
#define CELL_WIDTH 0.305
#define STEP 0.1525
#define NUM_OBSTACLES 15
#define MAX_DISTANCE 161
#define MAX_OBSTACLES 25                        /* maximum number of obstacles */
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  /*Used to round doubles to longs*/

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



double startInitial[2] = {0.305 * 14, 0.305 * 9};        /* start location */
double goalInitial[2] = {0.305 * 3, 0.305 * 2};         /* goal location */
long startFinal[2];                              /* start location translated to block location */
long goalFinal[2] ;                              /* goal location translated to block location*/
long obstacleFinal[NUM_OBSTACLES][2];            /*holds block locations of obstacles*/
struct Map Map1[FIELD_M][FIELD_N];              /*Structure array to be used for the map*/

int route[20];				/*Used to hold final directions for the path*/

/*Initialize structure values*/
int Initialize(void){

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


/* obstacle locations */
double obstacleInitial[NUM_OBSTACLES][2] =
{{0.305 * 3, 0.305 * 4},{0.305 * 4, 0.305 * 4},{0.305 * 5, 0.305 * 4},{0.305 * 3, 0.305 * 7},{0.305 * 4, 0.305 * 7},
{0.305 * 5, 0.305 * 7},{0.305 * 8, 0.305 * 2},{0.305 * 8, 0.305 * 3},{0.305 * 8, 0.305 * 4},{0.305 * 8, 0.305 * 7},
{0.305 * 9, 0.305 * 7},{0.305 * 10, 0.305 * 7},{0.305 * 11, 0.305 * 7},{0.305 * 11, 0.305 * 3},{0.305 * 11, 0.305 * 4},};

/*
*Converting Initial locations to block values
*/
int ConvertLocations(void){
    int i;

    /*Changing values for obstacles*/
    for (i=0; i<NUM_OBSTACLES; i++){
        //obstacleAns1 = obstacleInitial[i][0]/CELL_WIDTH;
        obstacleFinal[i][1] = round(obstacleInitial[i][0]/CELL_WIDTH)-1;
        obstacleFinal[i][0] = 10-round(obstacleInitial[i][1]/CELL_WIDTH);
    	//printf("Inside ConvertLocations.. Adding obstacles\n");
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
int PlaceObject(void){
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
    printf("Starting Manhattan \n");

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
			    /*printf("Updating complete cells from cell %d %d\n",i ,j);*/
			    if (i+1 <10){
                        	Map1[i+1][j].complete = 1;	/*Down*/
				/*printf("flag is now %d\n", Map1[i+1][j].complete);
				printf("Down (%d)(%d)\n",i+1,j);*/
			    }
			    if (i-1 >=0){
				Map1[i-1][j].complete = 1;	/*Up*/
				/*printf("flag is now %d\n", Map1[i-1][j].complete);
				printf("Up (%d)(%d)\n", i-1,j);*/
			    }
			    if (j+1 < 16){
		    		Map1[i][j+1].complete = 1;	/*Right*/
				/*printf("flag is now %d\n", Map1[i][j+1].complete);
				printf("Right (%d)(%d)\n", i, j+1);*/
			    }
			    if(j-1 >= 0){
				Map1[i][j-1].complete = 1;	/*Left*/
				/*printf("flag is now %d\n", Map1[i][j-1].complete);
				printf("Left (%d)(%d)\n",i,j-1);*/
			    }
		    }
	    }
	}
	/*Adjusting Available flag for completed cells (removes location from the next value update)*/
        for (i = 0; i < FIELD_M; i++) {
            for (j = 0; j < FIELD_N; j++) {
                    if (Map1[i][j].complete == 1 && Map1[i][j].noTouch == 0) {
			/*Completing surrounding cells*/
			    /*printf("UPDATING AVAILABLE IN CELL (%d) (%d)\n",i ,j);*/
                        	Map1[i][j].available  = 0;
				Map1[i][j].noTouch = 1;
		    }
            }
        }


	/*printf("\n");*/
        counter += 1;

        if (Map1[0][0].complete == 1){
            exit = 1;
	    /*printf("exiting");*/
        }
    }


        for (i = 0; i < FIELD_M; i++) {
		printf("\n");
            for (j = 0; j < FIELD_N; j++) {

		    printf("%3d ",Map1[i][j].val);
	    }
	}

    printf("\n");
    return 1;
}

/*
 *	Starting here is the algorithm for finding a path
 *	through the Manhattan populated map for the robot
 *	to follow.
 * */

int PathFinder(void){

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

	for (i=0; i<20; i++){
		route[i] = 0;
	}

	printf("%d \n",Place.val);

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
				printf("No Direction");
		}
		count ++;
		printf("Direction = %d \n", route[count-1]);
		printf("Current square value is: %d \n",Place.val);

	}

	printf("Path List\n");
	for (i=0; i<20; i++){
		printf("%d, ",route[i]);
	}
	printf("\n");
}






int main(void){

	Initialize();
	ConvertLocations();
	PlaceObject();
	Manhattan();
	PathFinder();
	return 0;
}
