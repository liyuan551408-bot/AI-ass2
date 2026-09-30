#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include "fuzzylogic.h"

using namespace std;

/////////////////////////////////////////////////////////////////

//Initialise Fuzzy Rules

void initFuzzyRules(fuzzy_system_rec *fl) {
	// main.cpp supplies Yamakawa-style combined inputs:
	// INPUT_X = A * theta + B * theta_dot
	// INPUT_Y = C * x     + D * x_dot
	// Build one complete 5-by-5 rule table over those two inputs.
	int rule_index = 0;
	for (int x_set = 0; x_set < fl->no_of_inp_regions; ++x_set) {
		for (int y_set = 0; y_set < fl->no_of_inp_regions; ++y_set) {
			rule& current = fl->rules[rule_index++];
			current.inp_index[0] = INPUT_X;
			current.inp_index[1] = INPUT_Y;
			current.inp_fuzzy_set[0] = x_set;
			current.inp_fuzzy_set[1] = y_set;

			// Positive pole error requests positive force. Positive cart error
			// requests negative force. The difference spans all nine outputs.
			current.out_fuzzy_set = out_ze + x_set - y_set;
		}
	}
}


void initMembershipFunctions(fuzzy_system_rec *fl) {
	// Both combined inputs use the same normalized universe. The left and
	// right shoulder sets also cover values outside [-2, 2].
	for (int input = 0; input < fl->no_of_inputs; ++input) {
		fl->inp_mem_fns[input][in_nl] =
			init_trapz(-2.0f, -1.0f, 0.0f, 0.0f, left_trapezoid);
		fl->inp_mem_fns[input][in_ns] =
			init_trapz(-2.0f, -1.0f, -0.5f, 0.0f, regular_trapezoid);
		fl->inp_mem_fns[input][in_ze] =
			init_trapz(-1.0f, -0.5f, 0.5f, 1.0f, regular_trapezoid);
		fl->inp_mem_fns[input][in_ps] =
			init_trapz(0.0f, 0.5f, 1.0f, 2.0f, regular_trapezoid);
		fl->inp_mem_fns[input][in_pl] =
			init_trapz(1.0f, 2.0f, 0.0f, 0.0f, right_trapezoid);
	}
}

void initFuzzySystem (fuzzy_system_rec *fl) {

	// The controller consumes the two combined inputs produced in main.cpp.
	fl->no_of_inputs = 2;
	fl->no_of_rules = 25;
	fl->no_of_inp_regions = 5;
	fl->no_of_outputs = 9;

	// Initial input scaling; tune these together with the membership ranges
	// after observing the simulation.
	coefficient_A = 1.3f;
	coefficient_B = 0.35f;
	coefficient_C = 0.6f;
	coefficient_D = 0.25f;

	// Nine symmetric force levels, aligned with NVL through PVL.
	for (int output = 0; output < fl->no_of_outputs; ++output) {
		fl->output_values[output] = -7.0f + 1.75f * output;
	}

	// The global fuzzy system is zero-initialized. Release an earlier rule
	// table if this initializer is called again.
	if (fl->allocated && fl->rules != NULL) {
		free(fl->rules);
	}
	fl->rules = static_cast<rule *>(malloc(
		static_cast<size_t>(fl->no_of_rules) * sizeof(rule)));
	fl->allocated = (fl->rules != NULL);
	if (!fl->allocated) {
		fl->no_of_rules = 0;
		return;
	}

	initFuzzyRules(fl);
	initMembershipFunctions(fl);
}

//////////////////////////////////////////////////////////////////////////////

trapezoid init_trapz (float x1,float x2,float x3,float x4, trapz_type typ) {
	
   trapezoid trz;
   trz.a = x1;
   trz.b = x2;
   trz.c = x3;
   trz.d = x4;
   trz.tp = typ;
   switch (trz.tp) {
	   
      case regular_trapezoid:
         	 trz.l_slope = 1.0/(trz.b - trz.a);
         	 trz.r_slope = 1.0/(trz.c - trz.d);
         	 break;
	 
      case left_trapezoid:
         	 trz.r_slope = 1.0/(trz.a - trz.b);
         	 trz.l_slope = 0.0;
         	 break;
	 
      case right_trapezoid:
         	 trz.l_slope = 1.0/(trz.b - trz.a);
         	 trz.r_slope = 0.0;
         	 break;
   }  /* end switch  */
   
   return trz;
}  /* end function */

//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
float trapz (float x, trapezoid trz) {
   switch (trz.tp) {
	   
      case left_trapezoid:
         	 if (x <= trz.a)
         	    return 1.0;
         	 if (x >= trz.b)
         	    return 0.0;
         	 /* a < x < b */
         	 return trz.r_slope * (x - trz.b);
	 
	 
      case right_trapezoid:
         	 if (x <= trz.a)
         	    return 0.0;
         	 if (x >= trz.b)
         	    return 1.0;
         	 /* a < x < b */
         	 return trz.l_slope * (x - trz.a);
	 
      case regular_trapezoid:
         	 if ((x <= trz.a) || (x >= trz.d))
         	    return 0.0;
         	 if ((x >= trz.b) && (x <= trz.c))
         	    return 1.0;
         	 if ((x >= trz.a) && (x <= trz.b))
         	    return trz.l_slope * (x - trz.a);
         	 if ((x >= trz.c) && (x <= trz.d))
         	    return  trz.r_slope * (x - trz.d);
         	    
	 }  /* End switch  */
	 
   return 0.0;  /* should not get to this point */
}  /* End function */

//////////////////////////////////////////////////////////////////////////////
float min_of(float values[],int no_of_inps) {
   int i;
   float val;
   val = values [0];
   for (i = 1;i < no_of_inps;i++) {
       if (values[i] < val)
	  val = values [i];
   }
   return val;
}



//////////////////////////////////////////////////////////////////////////////
float fuzzy_system (float inputs[],fuzzy_system_rec fz) {
   int i,j;
   short variable_index,fuzzy_set;
   float sum1 = 0.0,sum2 = 0.0,weight;
   float m_values[MAX_NO_OF_INPUTS];
	
   
   for (i = 0;i < fz.no_of_rules;i++) {
      for (j = 0;j < fz.no_of_inputs;j++) {
	   variable_index = fz.rules[i].inp_index[j];
	   fuzzy_set = fz.rules[i].inp_fuzzy_set[j];
	   m_values[j] = trapz(inputs[variable_index],
	       fz.inp_mem_fns[variable_index][fuzzy_set]);
	   } /* end j  */
      
       weight = min_of (m_values,fz.no_of_inputs);
				
       sum1 += weight * fz.output_values[fz.rules[i].out_fuzzy_set];
       sum2 += weight;
   } /* end i  */
 
	
	if (fabs(sum2) < TOO_SMALL) {
	  cout << "\r\nFLPRCS Error: Sum2 in fuzzy_system is 0.  Press key: " << endl;
      //~ getch();
      //~ exit(1);
      return 0.0;
   }
   
   return (sum1/sum2);
}  /* end fuzzy_system  */

//////////////////////////////////////////////////////////////////////////////
void free_fuzzy_rules (fuzzy_system_rec *fz) {
	if (fz->allocated && fz->rules != NULL) {
		free(fz->rules);
	}
	fz->rules = NULL;
	fz->allocated = false;
}

