/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <stdio.h>

/*
 * CONSTANTS
 */
#define FIELD_M 10                              /*Number of rows in the field*/
#define FIELD_N 16                              /*Number of columns in the field*/
#define CELL_WIDTH 0.305
#define STEP 0.1525
#define NUM_OBSTACLES 13
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


double startInitial[2] = {0.305, 1.219};        /* start location */
double goalInitial[2] = {3.658, 1.829};         /* goal location */
long startFinal[2];                              /* start location translated to block location */
long goalFinal[2] ;                              /* goal location translated to block location*/
long obstacleFinal[NUM_OBSTACLES][2];            /*holds block locations of obstacles*/
long filledMap[FIELD_M][FIELD_N];                /*Array of final values after Manhattan propagation*/
struct Map Map1[FIELD_M][FIELD_N];              /*Structure array to be used for the map*/

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
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
{3.353, 2.743}};

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
    
	return 1;
}

int FillMap(void){
    int i,j;

    for (i = 0; i < FIELD_M; i++) {
            printf("\n");
        for (j = 0; j < FIELD_N; j++) {
            filledMap[i][j] = Map1[i][j].val;
            printf(" %3d",filledMap[i][j] );
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
	int dir;
	char Route[25][6];
	struct Placement{
		long val;
		long position[2];
	};

	Placement Place;

	Place.position[0] = startFinal[0];
	Place.position[1] = startFinal[1];
	Place.val = filledMap[Place.position[0]][Place.position[1]];
	
	while(Place.val != -1){
		/*compare up and right*/
		if (filledMap[Place.position[0]-1][Place.position[1]] < filledMap[Place.position[0]][Place.position[1]+1]){
			dir = 1; 
			Place.val = filledMap[Place.position[0]-1][Place.position[1]];
			Place.position[0] -= 1;
		}	
		else{
			dir = 2;
			Place.val = filledMap[Place.position[0]][Place.position[1]+1];
			Place.position[1] += 1;
		}

		/*compare with down*/
		if (Place.val > filledMap[Place.position[0]+1][Place.position[1]]){
			dir = 3; 
			Place.val = filledMap[Place.position[0]+1][Place.position[1]];
			Place.position[0] += 1;
		}	

		/*compare with left */
		if (Place.val > filledMap[Place.position[0]][Place.position[1]-1]){
			dir = 4; 
			Place.val = filledMap[Place.position[0]][Place.position[1]-1];
			Place.position[1] -= 1;
		}	
		
		switch(dir){
		
			case 1:
				strcpy(route[count], "North");
				break;
			case 2:
				strcpy(route[count], "East");
				break;
			case 3:
				strcpy(route[count], "South");
				break;
			case 4:
				strcpy(route[count], "West");
				break;
			default:
				printf("No Direction");
		}
		count ++;
	}
	 

}






int main(void){

	Initialize();
	ConvertLocations();
	PlaceObject();
	Manhattan();
	FillMap();
	return 0;
}
