#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include "fuzzylogic.h"

using namespace std;

/////////////////////////////////////////////////////////////////

//Initialise Fuzzy Rules

void initFuzzyRules(fuzzy_system_rec *fl) {
	// INPUT_X = A * theta + B * theta_dot
	// INPUT_Y = C * x     + D * x_dot
	// Rows: X = NM, NS, ZR, PS, PM; columns: Y in the same order.
	const short output_table[5][5] = {
		{out_nl, out_nl, out_nm, out_nm, out_ns},
		{out_nm, out_nm, out_ns, out_ns, out_ze},
		{out_ns, out_ns, out_ze, out_ps, out_ps},
		{out_ze, out_ps, out_ps, out_pm, out_pm},
		{out_ps, out_pm, out_pm, out_pl, out_pl}
	};

	int rule_index = 0;
	
	for (int x_set = 0; x_set < fl->no_of_inp_regions; ++x_set) {
		for (int y_set = 0; y_set < fl->no_of_inp_regions; ++y_set) {
			rule& current = fl->rules[rule_index++];
			current.inp_index[0] = INPUT_X;
			current.inp_index[1] = INPUT_Y;
			current.inp_fuzzy_set[0] = x_set;
			current.inp_fuzzy_set[1] = y_set;

			current.out_fuzzy_set = output_table[x_set][y_set];
		}
	}
}


void initMembershipFunctions(fuzzy_system_rec *fl) {

    // INPUT X = A*theta + B*theta_dot
    // calibrated universe: approximately [-4, 4]

    fl->inp_mem_fns[INPUT_X][in_nm] =init_trapz(-4.0f, -2.0f, 0.0f, 0.0f,left_trapezoid);

    fl->inp_mem_fns[INPUT_X][in_ns] =init_trapz(-4.0f, -2.0f, -2.0f, 0.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_X][in_ze] =init_trapz(-2.0f, 0.0f, 0.0f, 2.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_X][in_ps] =init_trapz(0.0f, 2.0f, 2.0f, 4.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_X][in_pm] =init_trapz(2.0f, 4.0f, 0.0f, 0.0f,right_trapezoid);

    // INPUT Y = C*x + D*x_dot
    // Start with the same assignment-suggested universe [-4, 4].

    fl->inp_mem_fns[INPUT_Y][in_nm] =init_trapz(-4.0f, -2.0f, 0.0f, 0.0f,left_trapezoid);

    fl->inp_mem_fns[INPUT_Y][in_ns] =init_trapz(-4.0f, -2.0f, -2.0f, 0.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_Y][in_ze] =init_trapz(-2.0f, 0.0f, 0.0f, 2.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_Y][in_ps] =init_trapz(0.0f, 2.0f, 2.0f, 4.0f,regular_trapezoid);

    fl->inp_mem_fns[INPUT_Y][in_pm] =init_trapz(2.0f, 4.0f, 0.0f, 0.0f,right_trapezoid);
}

void initFuzzySystem (fuzzy_system_rec *fl) {

	// Yamakawa controller:
    // two combined inputs X and Y
	fl->no_of_inputs = 2;
	// extending Yamakawa's 13 rules to 25.
	fl->no_of_rules = 25;
	// NM, NS, ZE, PS, PM
	fl->no_of_inp_regions = 5;
	// NL, NM, NS, ZE, PS, PM, PL
	fl->no_of_outputs = 7;

	// Positive constants, empirically calibrated.
	coefficient_A = 3.9f;
	coefficient_B = 0.80f;
	coefficient_C = 0.25f;
	coefficient_D = 0.75f;

    // final calibration values.
    fl->output_values[out_nl] = -100.0f;
    fl->output_values[out_nm] =  -60.0f;
    fl->output_values[out_ns] =  -30.0f;
    fl->output_values[out_ze] =    0.0f;
    fl->output_values[out_ps] =   30.0f;
    fl->output_values[out_pm] =   60.0f;
    fl->output_values[out_pl] =  100.0f;

	// The global fuzzy system is zero-initialized. Release an earlier rule
	// table if this initializer is called again.
	if (fl->allocated && fl->rules != NULL) {
		free(fl->rules);
	}

	fl->rules = static_cast<rule *>(malloc(static_cast<size_t>(fl->no_of_rules) * sizeof(rule)));
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

