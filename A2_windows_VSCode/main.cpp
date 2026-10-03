///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    ARTIFICIAL INTELLIGENCE 159302
//    INVERTED PENDULUM SIMULATION
// 
//	  Description: Inverted pendulum simulation with fuzzy logic engine and animation functions
//
//    Run Parameters: 
//
//    Keys for Operation: 
//
//
//    Start-up code by:  Dr. Napoleon Reyes, n.h.reyes@massey.ac.nz
//    					 Computer Science, SMCS
//						 Massey University-Albany
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <windows.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <iostream>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <stdexcept>


#include "sprites.h" 
#include "graphics.h"
#include "transform.h"
#include "fuzzylogic.h"

using namespace std;


/// Global Variables ///////////////////////////////////////////////////////////////////////


bool DEBUG_MODE=false;
float WORLD_MAXX, WORLD_MAXY;
int fieldX1, fieldY1, fieldX2, fieldY2; //playing field boundaries
BoundaryType worldBoundary,deviceBoundary;
char keyPressed[5];
fuzzy_system_rec g_fuzzy_system;
float coefficient_A, coefficient_B, coefficient_C, coefficient_D;



struct WorldStateType{
	
	void init(){
		x=0.0;
		x_dot=0.0;
		x_double_dot = 0.0;
		angle = 0.0;
		angle_dot = 0.0;
		angle_double_dot = 0.0;
		F = 0.0;
	}
	
	float x;
	float x_dot;
	float x_double_dot;
	float angle;
	float angle_dot;
	float angle_double_dot;
	
	float const mb=0.1;
	float const g=9.8;
	float const m=1.1; // mass of cart & broom
	float const l=0.5;
	
	float F;
};


struct DataSetType{
   vector<float> x;
   vector<float> y;
   vector<vector<float> > z; 
};

DataSetType dataSet;

int NUM_OF_DATA_POINTS;
const float PI = 3.14159265358979323846f;
const float INITIAL_CART_X = 1.0f;

// Function Prototypes ////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////
float getKey() {
   
	  float F=0.0;
	
     if(GetAsyncKeyState(VK_LEFT) < 0) {     
        //"LEFT ARROW";
		  F=-7.0;
	  }
	  
	  if(GetAsyncKeyState(VK_RIGHT) < 0) { 
        F=7.0;  
    
        //"RIGHT ARROW"
	  }
	  
	  return F;
}

////////////////////////////////////////////////////////////////////////////////



void initPendulumWorld(){
	
	//widescreen
   fieldX1 = getmaxx() / 10;
   fieldX2 = getmaxx() - (getmaxx() / 10);
   fieldY1 = getmaxy() / 9;
   fieldY2 = getmaxy() - (getmaxy() / 9);
    
  
    worldBoundary.x1 = -2.4;
    //worldBoundary.y1 = 1.2;
	worldBoundary.y1 = 3;
    worldBoundary.x2 = 2.4;
    worldBoundary.y2 = -0.4;

    deviceBoundary.x1 = fieldX1;
    deviceBoundary.y1 = fieldY1;
    deviceBoundary.x2 = fieldX2;
    deviceBoundary.y2 = fieldY2;
       
    WORLD_MAXX=worldBoundary.x2-worldBoundary.x1;
    WORLD_MAXY=worldBoundary.y2-worldBoundary.y1;
	
}

void drawInvertedPendulumWorld(){
	
	setcolor(WHITE);
	rectangle(xDev(worldBoundary,deviceBoundary,worldBoundary.x1),yDev(worldBoundary,deviceBoundary,worldBoundary.y1),
	          xDev(worldBoundary,deviceBoundary,worldBoundary.x2),yDev(worldBoundary,deviceBoundary,worldBoundary.y2));
   //~ setcolor(YELLOW);
	//~ rectangle(xDev(worldBoundary,deviceBoundary,worldBoundary.x1),yDev(worldBoundary,deviceBoundary,worldBoundary.y2+0.07),
	          //~ xDev(worldBoundary,deviceBoundary,worldBoundary.x2),yDev(worldBoundary,deviceBoundary,worldBoundary.y2));
	settextstyle(TRIPLEX_FONT, HORIZ_DIR, 2);
	settextjustify(CENTER_TEXT, CENTER_TEXT);
	
	setcolor(WHITE);
	outtextxy((deviceBoundary.x1 + deviceBoundary.x2)/2, deviceBoundary.y1 - 3 * textheight("H"),"ARTIFICIAL INTELLIGENCE 159302");
    settextstyle(TRIPLEX_FONT, HORIZ_DIR, 1);
	outtextxy((deviceBoundary.x1 + deviceBoundary.x2)/2, deviceBoundary.y1 - 2 * textheight("H"),"INVERTED PENDULUM");
	settextstyle(TRIPLEX_FONT, HORIZ_DIR, 1);
	outtextxy((deviceBoundary.x1 + deviceBoundary.x2)/2, deviceBoundary.y1 - textheight("H"),"FUZZY LOGIC CONTROLLER");
	settextstyle(SMALL_FONT, HORIZ_DIR, 5);
	outtextxy((deviceBoundary.x2 - textwidth("START-UP CODE by:  n.h.reyes@massey.ac.nz")), deviceBoundary.y2 + textheight("H"),"START-UP CODE by:  n.h.reyes@massey.ac.nz  (C) Massey University 2026");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BEGIN - DYNAMICS OF THE SYSTEM
float calc_angular_acceleration( const WorldStateType& s){		
	float a_double_dot=0.0;
	float numerator=0.0;
	float denominator=0.0;

	numerator = (s.m * s.g * sin(s.angle) - (cos(s.angle) * (s.F + ((s.mb) * s.l * s.angle_dot * s.angle_dot * sin(s.angle) ) ) )  );
	denominator = (  ((4/3)*s.m * s.l) - (s.mb * s.l * cos(s.angle) * cos(s.angle)));
	if(numerator == 0.0 || denominator == 0.0){
	   a_double_dot=0.0;	
	} else {
	   a_double_dot = numerator/denominator;
    }  

	return a_double_dot;
}

float calc_horizontal_acceleration( const WorldStateType& s){	
	float x_double_dot=0.0;
	float numerator=0.0;


	numerator = ( s.F + s.mb * s.l * (s.angle_dot * s.angle_dot)* sin(s.angle) - s.angle_double_dot * cos(s.angle)   );
	if(numerator == 0.0){
        x_double_dot=0.0;
	} else {
	    x_double_dot = numerator / s.m;	
    }
	return x_double_dot; 
}
// END - DYNAMICS OF THE SYSTEM
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void welcomeScreen(){
	drawInvertedPendulumWorld();
	
	
	settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);
    setcolor(YELLOW);
    settextjustify(LEFT_TEXT, CENTER_TEXT);

    string msg = "Getting started...";
	outtextxy((deviceBoundary.x1 + textwidth("N")), deviceBoundary.y1 + (8*textheight("H")),msg.c_str());
	setcolor(GREEN);
    msg = "Please click mouse on Command prompt";
	outtextxy((deviceBoundary.x1 + textwidth("N")), deviceBoundary.y1 + (10*textheight("H")),msg.c_str());
	msg = "window, then type the initial angle.";
	outtextxy((deviceBoundary.x1 + textwidth("N")), deviceBoundary.y1 + (11*textheight("H")),msg.c_str());
}

void displayInfo(const WorldStateType& s, const string msg="", const string timeElapsed=""){
	setcolor(WHITE);
	outtextxy((deviceBoundary.x1 + deviceBoundary.x2)/2, deviceBoundary.y1 - 2 * textheight("H"),"INVERTED PENDULUM");
	settextstyle(TRIPLEX_FONT, HORIZ_DIR, 1);
	outtextxy((deviceBoundary.x1 + deviceBoundary.x2)/2, deviceBoundary.y1 - textheight("H"),"FUZZY LOGIC CONTROLLER");
	settextstyle(SMALL_FONT, HORIZ_DIR, 6);
	
	char angleStr[120];
	char xStr[120];
	
	float a=((s.angle*180/3.14));
	
	if(a > 360){
		a = a / 360.0;
	}
	
	sprintf(xStr,"x = %4.2f",s.x);
	outtextxy((deviceBoundary.x2 - textwidth("n.h.reyes@massey.ac.nz")), deviceBoundary.y2 - (7*textheight("H")),xStr);
	
	sprintf(angleStr,"angle = %4.2f",a);
	outtextxy((deviceBoundary.x2 - textwidth("n.h.reyes@massey.ac.nz")), deviceBoundary.y2 - (6*textheight("H")),angleStr);
	
	sprintf(angleStr,"F = %4.2f",s.F);
	outtextxy((deviceBoundary.x2 - textwidth("n.h.reyes@massey.ac.nz")), deviceBoundary.y2 - (5*textheight("H")),angleStr);
    
    if(!msg.empty()){
    	settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);
    	setcolor(GREEN);
    	settextjustify(LEFT_TEXT, CENTER_TEXT);
	    outtextxy((deviceBoundary.x1 + textwidth("TIME")), deviceBoundary.y1 + (3*textheight("H")),msg.c_str());
    }

    if(!timeElapsed.empty()){
    	settextstyle(DEFAULT_FONT, HORIZ_DIR, 3);
    	setcolor(WHITE);
    	settextjustify(LEFT_TEXT, CENTER_TEXT);
	    outtextxy((deviceBoundary.x1 + textwidth("TIME")), deviceBoundary.y1 + (5*textheight("H")),timeElapsed.c_str());
    }

}




void runInvertedPendulum(){
	
	using std::chrono::steady_clock;

	// float inputs[4];
	float inputs[2];  //Yamakawa
	
	WorldStateType prevState, newState;
	// srand(time(NULL));  // Seed the random number generator
			
    initPendulumWorld();
	
	static bool page;
	
	float const h=0.002;
	float externalForce=0.0;
	
	prevState.init();
	newState.init();
	//-------------------------------------------------

	welcomeScreen();
	
	//-------------------------------------------------
	//Start at the origin
	//Cart cart(0.0, worldBoundary.y2 + 0.06);
	//Rod rod(0.0, worldBoundary.y2 + 0.06);
	//-------------------------------------------------
	
	// The assignment characterises controller performance at x=1 m.
	//Cart cart(-1.0, worldBoundary.y2 + 0.06);
	Cart cart(INITIAL_CART_X, worldBoundary.y2 + 0.125);
	//Rod rod(-1.0, worldBoundary.y2 + 0.06);
	Rod rod(INITIAL_CART_X, worldBoundary.y2 + 0.125 + 0.35);
	
	//---------------------------------------------------------------
    //***************************************************************
    //Set the initial angle of the pole with respect to the vertical
    prevState.x = INITIAL_CART_X;
	prevState.angle = 25.0 * (3.14/180);  //initial angle  = 35 degrees
	
	
    initFuzzySystem(&g_fuzzy_system);	
	if (!g_fuzzy_system.allocated) {
		throw runtime_error("Unable to allocate fuzzy controller rules.");
	}
	
	//~ display_All_MF (g_fuzzy_system);
    //~ getch();
	steady_clock::time_point start;
	
    string msg;
    bool exitFlag=false;

    
    float input_angle=0;

        while(true){
        	exitFlag=false;
            msg.clear();
            
            prevState.init();
            newState.init();
            prevState.x = INITIAL_CART_X;

		    cout << "Enter initial angle [-60, 60], (to exit, leave it blank): ";
		    

		    std::string input;
		    if (!std::getline(std::cin, input) ||
		        input.find_first_not_of(" \t\r\n") == string::npos) {
		        break;
		    }
		    std::istringstream stream(input);
		    if (!(stream >> input_angle)) {
		        cout << "Please enter a number in [-60, 60]." << endl;
		        continue;
		    }
		    stream >> std::ws;
		    if (!stream.eof() || !std::isfinite(input_angle) ||
		        input_angle < -60.0f || input_angle > 60.0f) {
		        cout << "Please enter a number in [-60, 60]." << endl;
		        continue;
		    }

		    if(input_angle == 0.0){ //perturb by 0.1 degrees if initial angle is set to 0
		        input_angle = 0.1f * (PI / 180.0f);
		    } else {
		        input_angle = input_angle * (PI / 180.0f);
		    }


		    prevState.angle = input_angle;
		    newState.x = prevState.x;
		    newState.angle = prevState.angle;

			start = steady_clock::now();
			double simulationTime = 0.0;
			cout << "Left/Right: apply a disturbance; Esc: end this trial." << endl;

			while((GetAsyncKeyState(VK_ESCAPE) & 0x8000) == 0) {

		         setactivepage(page);
		         cleardevice();
				 drawInvertedPendulumWorld();
			
			     //retrieve inputs
				 // inputs[in_theta] = prevState.angle;
				 // inputs[in_theta_dot] = prevState.angle_dot;
				 // inputs[in_x] = prevState.x;
				 // inputs[in_x_dot] = prevState.x_dot;


				 inputs[INPUT_X] = (coefficient_A * prevState.angle) + (coefficient_B * prevState.angle_dot);
				 inputs[INPUT_Y] = (coefficient_C * prevState.x) + (coefficient_D * prevState.x_dot);
				
		         // Calculate a fresh automatic force at every simulation step.
		         prevState.F = fuzzy_system(inputs, g_fuzzy_system);
				 
				 externalForce=0.0;
				 externalForce = getKey(); //manual operation
				 
				 if(externalForce != 0.0)
				 	prevState.F = externalForce;
		         
		         if(DEBUG_MODE){
		           cout << "F = " << prevState.F << endl; //for debugging purposes only
		         }
				
				 //---------------------------------------------------------------------------
				 // **************************************************************************
				 // BEGIN - DYNAMICS OF THE SYSTEM

				 //Calculate the new state of the world
				 newState.angle_double_dot = calc_angular_acceleration(prevState);
				 newState.angle_dot = prevState.angle_dot + (h * newState.angle_double_dot); 
				 newState.angle = prevState.angle + (h * newState.angle_dot);
				 newState.F = prevState.F;				 
				 newState.x_double_dot = calc_horizontal_acceleration(prevState);
				 newState.x_dot = prevState.x_dot + (h * newState.x_double_dot);
		         newState.x = prevState.x + (h * newState.x_dot);

		         // cout << "newState.angle = " << newState.angle << endl;
		         // cout << "newState.angle_dot = " << newState.angle_dot << endl;
				 
				 if(DEBUG_MODE){
			         cout << "prevState.angle = " << prevState.angle << endl;
					 cout << "prevState.angle_dot = " << prevState.angle_dot << endl;
					 cout << "prevState.angle_double_dot = " << prevState.angle_double_dot << endl;
					 cout << "prevState.x = " << prevState.x << endl;
					 cout << "prevState.x_dot = " << prevState.x_dot << endl;
					 cout << "prevState.x_double_dot = " << prevState.x_double_dot << endl;
				 }
				 
				 
				 prevState.x = newState.x;		
		 		 prevState.angle = newState.angle;
				 prevState.x_dot = newState.x_dot;
				 prevState.angle_dot = newState.angle_dot;		
				 prevState.angle_double_dot = newState.angle_double_dot;
				 prevState.x_double_dot = newState.x_double_dot;
				 //--------------------------	 		 
				 cart.setX(newState.x);
				 rod.setX(newState.x);
				 rod.setAngle(newState.angle);		 
				 cart.draw();
				 rod.draw();
				 // END - DYNAMICS OF THE SYSTEM
				 // **************************************************************************
				 //---------------------------------------------------------------------------
				 simulationTime += h;
				 

				 if((prevState.x < (-2.4 + 0.3)) || (prevState.x > (2.4-0.3))){
		 	 	    msg =  "Cart-pole went out of bounds.";
		            exitFlag = true;
			
		         } else if((((prevState.angle*180/3.14)) < -90) || (((prevState.angle*180/3.14)) > 90)) {
				 	msg = "pole fell down.";         	
				 	exitFlag = true;
		         
		         } else {
		         	displayInfo(newState);
		         
		         }

				 setvisualpage(page);

			     if(exitFlag) break;

		         page = !page;  //switch to another page
		         


		         // getch();
		         if(mousedown()){
		         	//do nothing
		         }
		   }

    auto end = steady_clock::now();

	std::chrono::duration<double> elapsed_seconds = end-start;
	if (!exitFlag) {
		msg = "Trial stopped (Esc).";
	}
	ostringstream timing;
	timing << fixed << setprecision(2) << simulationTime << " sim. sec.";
	string timeStr = timing.str();
	cout << msg << " Simulation time: " << simulationTime
	     << " sec.; elapsed time: " << elapsed_seconds.count() << " sec." << endl;
	
	// cout << "timeElapsed = " << elapsed_seconds.count() << endl;
	displayInfo(newState, msg, timeStr);         	
	setvisualpage(page);
	// getch();
	}

		
	
	free_fuzzy_rules(&g_fuzzy_system);
}


void generateControlSurface_Angle_vs_Angle_Dot(){		
	float inputs[2]; //Yamakawa

	cout << "Generating control surface (Angle vs. Angle_Dot)..." << endl;
	WorldStateType prevState, newState;
	srand(time(NULL));  // Seed the random number generator
			
    initPendulumWorld();
	
	static bool page;
	
	float const h=0.002;
	
	prevState.init();
	newState.init();
	

	//-------------------------------------------------

	//~ Cart cart(-1.0, worldBoundary.y2 + 0.125);
	//~ Rod rod(-1.0, worldBoundary.y2 + 0.125 + 0.35);
	//-------------------------------------------------
	
		
    initFuzzySystem(&g_fuzzy_system);	
	if (!g_fuzzy_system.allocated) {
		throw runtime_error("Unable to allocate fuzzy controller rules.");
	}
	
	
    float angle_increment;
    float angle_dot_increment;

    float minAngle=0;
    float maxAngle=0;
    float angle=0.0;

    float angle_dot=0.0;
    float minAngleDot=0;
    float maxAngleDot=0;

    NUM_OF_DATA_POINTS=100;

//---------------------------------
    dataSet.x.resize(NUM_OF_DATA_POINTS);
    dataSet.y.resize(NUM_OF_DATA_POINTS);
    dataSet.z.resize(NUM_OF_DATA_POINTS);

    for(int y=0; y < NUM_OF_DATA_POINTS; y++){
       dataSet.z[y].resize(NUM_OF_DATA_POINTS);	
    }
//---------------------------------    
    minAngleDot= -3.0;
    maxAngleDot=  3.0;
    angle_dot_increment=(maxAngleDot-minAngleDot)/float(NUM_OF_DATA_POINTS - 1);
    angle_dot=minAngleDot;
//---------------------------------
    minAngle=(-40.0f)*PI/180.0f;
    maxAngle=(40.0f)*PI/180.0f;
    angle_increment=(maxAngle-minAngle)/float(NUM_OF_DATA_POINTS - 1);

//---------------------------------
    for(int row=0; row < NUM_OF_DATA_POINTS; row++){
         angle_dot = minAngleDot + row * angle_dot_increment;
    	 dataSet.y[row] = angle_dot;
         prevState.angle_dot = angle_dot;
         angle=minAngle;

         for(int col=0; col < NUM_OF_DATA_POINTS; col++){
             angle = minAngle + col * angle_increment;
             prevState.x=0.0;
		     prevState.x_dot=0.0;
		     prevState.x_double_dot = 0.0;
		     prevState.angle = angle;
		     prevState.angle_dot = angle_dot;
		     prevState.angle_double_dot = 0.0;
		     prevState.F = 0.0; 


             dataSet.x[col] = angle;
	         prevState.angle = angle;

			 inputs[INPUT_X] = (coefficient_A * prevState.angle) + (coefficient_B * prevState.angle_dot);
			 inputs[INPUT_Y] = (coefficient_C * prevState.x) + (coefficient_D * prevState.x_dot);
			
	         prevState.F = 0.0;  //nothing is done.//fuzzy_system(inputs, g_fuzzy_system);
			
			
			 //---------------------------------------------------------------------------
			 //Calculate the new state of the world
			 //Updating angle
			 
			 newState.angle_double_dot = calc_angular_acceleration(prevState);
			 newState.angle_dot = prevState.angle_dot + (h * newState.angle_double_dot); 
			 newState.angle = prevState.angle + (h * newState.angle_dot);
			 newState.F = prevState.F;
			
			 //Updating x
					 
			 newState.x_double_dot = calc_horizontal_acceleration(prevState);
			 newState.x_dot = prevState.x_dot + (h * newState.x_double_dot);
	         newState.x = prevState.x + (h * newState.x_dot);

	//Update previous state
			 prevState.x = newState.x;		
	 		 prevState.angle = newState.angle;
			 prevState.x_dot = newState.x_dot;
			 prevState.angle_dot = newState.angle_dot;		
			 prevState.angle_double_dot = newState.angle_double_dot;
			 prevState.x_double_dot = newState.x_double_dot;
			 
			 inputs[INPUT_X] = (coefficient_A * prevState.angle) + (coefficient_B * prevState.angle_dot);
			 inputs[INPUT_Y] = (coefficient_C * prevState.x) + (coefficient_D * prevState.x_dot);
			
	         prevState.F = fuzzy_system(inputs, g_fuzzy_system);
			 dataSet.z[row][col] = prevState.F; //record Force calculated
			 
      }
   }	
   
   free_fuzzy_rules(&g_fuzzy_system);
   cout << "done collecting data." << endl;

}




void saveDataToFile(string fileName){
	cout << "Saving control surface to file: " << fileName << "..." << endl;
	ofstream myfile;
	myfile.exceptions(ofstream::failbit | ofstream::badbit);
	myfile.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc);
	myfile << setprecision(9);
	
	// Excel surface layout: columns = angle (rad), rows = angular velocity
	// (rad/s), cells = force (N). Clear the top-left zero in Excel before plotting.
	myfile << "0.00";
	for(int col=0; col < NUM_OF_DATA_POINTS; col++){
		myfile << "," << dataSet.x[col];
	}
	myfile << '\n';
	for(int row=0; row < NUM_OF_DATA_POINTS; row++){
	  myfile << dataSet.y[row];
	  for(int col=0; col < NUM_OF_DATA_POINTS; col++){
		  myfile << "," << dataSet.z[row][col];
     }
	  myfile << '\n';
   }
 
   myfile.close();
   cout << "Data set saved (File: " << fileName << ")" << endl;
	
}

void clearDataSet(){
	dataSet.x.clear();
	dataSet.y.clear();
	dataSet.z.clear();
	NUM_OF_DATA_POINTS = 0;
   cout << "DataSet cleared." << endl;
	
}


////////////////////////////////////////////////////////////////////////////////////

int main(void) {
	
   int graphDriver = 0,graphMode = 0;
   
   initgraph(&graphDriver, &graphMode, "", 800, 600); // Start Window
   int graphicsError = graphresult();
   if (graphicsError != grOk) {
       cerr << "Graphics initialization failed: "
            << grapherrormsg(graphicsError) << endl;
       return EXIT_FAILURE;
   }
   clearDataSet();
   int status = EXIT_SUCCESS;
   try{
		runInvertedPendulum();
	
		generateControlSurface_Angle_vs_Angle_Dot();
		saveDataToFile("data_angle_vs_angle_dot.txt");
		
   }
   catch(const std::exception& error){
       cerr << "Error: " << error.what() << endl;
       status = EXIT_FAILURE;
   }
   catch(...){
       cerr << "Unexpected error." << endl;
       status = EXIT_FAILURE;
   }
   free_fuzzy_rules(&g_fuzzy_system);
   closegraph();
	return status;
} 

