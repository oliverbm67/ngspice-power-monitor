/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE d_power_simulator/cfunc.mod

AUTHOR                     

    15 Aug 2019     Olivier Betschi


MODIFICATIONS   
                                   

SUMMARY

    This file contains the functional description of the digital power simulator


INTERFACES       

    FILE                 ROUTINE CALLED     

    CMevt.c              void *cm_event_alloc()
                         void *cm_event_get_ptr()


REFERENCED FILES

    Inputs from and outputs to ARGS structure.
                     

NON-STANDARD FEATURES

    NONE
NOTE TO AVOID CONFUSION

    The connection named out, is an INPUT of the system, and is connected to the OUTPUT of the device that is modelled. The OUTPUT of the power simulator is the power_control port.

===============================================================================*/

/*=== INCLUDE FILES ====================*/

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
                                      

/*=== CONSTANTS ========================*/




/*=== MACROS ===========================*/



  
/*=== LOCAL VARIABLES & TYPEDEFS =======*/                         


    
           
/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/




                   
/*==============================================================================

FUNCTION ucm_d_power_simulator()

AUTHORS                      

    15 Aug 2019     Olivier Betschi

MODIFICATIONS   



SUMMARY

    This function implements the power simulator code model

INTERFACES       

    FILE                 ROUTINE CALLED     

    CMevt.c              void *cm_event_alloc()
                         void *cm_event_get_ptr()

RETURNED VALUE
    
    Returns inputs and outputs via ARGS structure.

GLOBAL VARIABLES
    
    NONE

NON-STANDARD FEATURES

    NONE

==============================================================================*/

/*=== UCM_D_POWER_SIMULATOR ROUTINE ===*/

/************************************************
*      The following is the model for the       *
*   power simulator code model                  *
*                                               *
*   Created 15/08/19               O. Betschi   *
************************************************/


void ucm_d_power_simulator(ARGS) 

{
    int             i;   /* generic loop counter index */       
                        
    Digital_State_t *input,*input_old;  /* input variable */

    Digital_State_t *out, *out_old; /* output variable */

    bool            *active, *active_old; /* track the status of the activity */

    bool            *switch_input, *switch_output; /* track the switch of input and output */
    bool            *switch_input_old, *switch_output_old; /* track the switch of input and output */

    int             size_in, size_out;      /* size of the input and output ports */

    double          active_current, passive_current; /* control current */


    /* determine "width" of the input and output... */

    size_in = PORT_SIZE(in); 
    size_out = PORT_SIZE(out); 

    /* Read current parameters */

    active_current=PARAM(active_current);
    passive_current=PARAM(passive_current);
  

    /** Setup required state variables **/

    if(INIT) {  /* initial pass */ 

        /* Allocate storage for the input connection */
        cm_event_alloc(0,size_in*sizeof(Digital_State_t));
        /* Allocate storage for the output connection */
        cm_event_alloc(1,size_out*sizeof(Digital_State_t));

        /* The power output do not require any storage */

        /* Allocate storage for the switching */
        cm_event_alloc(2,sizeof(bool));
        cm_event_alloc(3,sizeof(bool));

        /* Allocate storage for active status */
        cm_event_alloc(4,sizeof(bool));

        /* Assign discrete address of inputs */
        input = input_old = (Digital_State_t *) cm_event_get_ptr(0,0);
        /* Assign discrete address of output */
        out = out_old = (Digital_State_t *) cm_event_get_ptr(1,0);

        /* Assign discrete addresses for switches */
        switch_input = switch_input_old = (bool *) cm_event_get_ptr(2,0);
        switch_output = switch_output_old = (bool *) cm_event_get_ptr(3,0);

        /* Assign discrete address for active status */
        active = active_old = (bool *) cm_event_get_ptr(4,0);

        /* Read input values */
        for (i=0; i<size_in;i++){
            input[i]=INPUT_STATE(in[i]);
            input_old[i]=INPUT_STATE(in[i]);
        }

        /* Read output values */
        for (i=0; i<size_out;i++){
            out[i]=INPUT_STATE(out[i]);
            out_old[i]=INPUT_STATE(out[i]);
        }
    }

    else {      /* Retrieve previous values -- no first pass*/

        /* Retrieve value of digital input */
        input = (Digital_State_t *) cm_event_get_ptr(0,0);
        input_old = (Digital_State_t *) cm_event_get_ptr(0,1);
        /* Retrieve value of digital output */
        out = (Digital_State_t *) cm_event_get_ptr(1,0);
        out_old = (Digital_State_t *) cm_event_get_ptr(1,1);
        /* Retrieve value of switching */
        switch_input = (bool *) cm_event_get_ptr(2,0);
        switch_input_old = (bool *) cm_event_get_ptr(2,1);
        switch_output = (bool *) cm_event_get_ptr(3,0);
        switch_output_old = (bool *) cm_event_get_ptr(3,1);
        /* Retrieve active status */
        active = (bool *) cm_event_get_ptr(4,0);
        active_old = (bool *) cm_event_get_ptr(4,1);

        /* Read input values */
        for (i=0; i<size_in;i++){
            input[i]=INPUT_STATE(in[i]);
        }

        /* Read output values */
        for (i=0; i<size_out;i++){
            out[i]=INPUT_STATE(out[i]);
        }

    }

    switch (CALL_TYPE) {

        case EVENT: /* event driven simulation */

            /* Look for modification of the input or input undetermined*/
            *switch_input =FALSE;
            for (i=0; i< size_in; i++) {
                if ( input[i] != input_old[i] || input[i] == UNKNOWN) *switch_input = TRUE;
            }

            /* Look for modification of the output or if output is undetermined */
            *switch_output =FALSE;
            for (i=0; i< size_out; i++) {
                if ( out[i] != out_old[i] || out[i] == UNKNOWN) *switch_output = TRUE;
            }

            /* The active signal is turned on when the input switches */
            if ( *switch_input == TRUE ) {
                *active = TRUE;
            }
            
            /* The active signal is turned off when output switches */
            if ( *active_old == TRUE && *switch_output_old == TRUE && *switch_output == FALSE && *switch_input == FALSE) {
                *active = FALSE;
            }
            
        break;

        case ANALOG:    /* analog simulation */

            if ( 0.0 == TIME ) {            /* DC analysis */
                if ( *active == TRUE ) {
                    OUTPUT(power_control) = active_current;
                }
                else {
                    OUTPUT(power_control) = passive_current;
                }

            }
            
            
            else {                           /* Transient analysis */

                if ( *active == TRUE ) {
                    OUTPUT(power_control) = active_current;
                }
                else {
                    OUTPUT(power_control) = passive_current;
                }

            }

        break;
    }

}



