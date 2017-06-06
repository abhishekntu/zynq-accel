#include "xil_types.h"
#include "xstatus.h"
#include "xil_testmem.h"
#include "xil_io.h"
#include "platform.h"
#include "time.h"
#include <stdlib.h>
#include "sleep.h"
#include "xil_assert.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "xil_mmu.h"
#include "xdmaps.h"
#include "time.h"
#include "math.h"
/***********************************************************
Timer base address
************************************************************/
#define timer_base 0xf8f00000
/***********************************************************
Basic definitions
************************************************************/
#define pi 3.141592653589793
#define SAMPLES 1000
/***********************************************************
Timer Registers
************************************************************/
static volatile int *timer_counter_l=(volatile int *)(timer_base+0x200);
static volatile int *timer_counter_h=(volatile int *)(timer_base+0x204);
static volatile int *timer_ctrl=(volatile int *)(timer_base+0x208);
/***********************************************************
Performance Measurement Functions definitions
************************************************************/
void init_timer(volatile int *timer_ctrl, volatile int *timer_counter_l, volatile int *timer_counter_h){
	*timer_ctrl=0x0;
	*timer_counter_l=0x1;
	*timer_counter_h=0x0;
	DATA_SYNC;
}

void start_timer(volatile int *timer_ctrl){
	*timer_ctrl=*timer_ctrl | 0x00000001;
	DATA_SYNC;
}

void stop_timer(volatile int *timer_ctrl){
	*timer_ctrl=*timer_ctrl & 0xFFFFFFFE;
	DATA_SYNC;
}
/*****************************************************************
Main function: Generating Input signal and FIR filter coefficients
Applying SW based FIR filtering on 1K samples
******************************************************************/
int main()
{
    init_platform();

    xil_printf("------------------------------\n\r");
    xil_printf("Fixed point FIR filtering on ARM Cortex-A9\n\r");
    xil_printf("------------------------------\n\r");
    /***************************************************************************
	Generation of Input signal for Fixed point FIR filtering on ARM Cortex-A9
    ****************************************************************************/
    int i,j;
    float input[SAMPLES];
    short input_fixed[1000];
    for(i=0;i<SAMPLES;i++){
    		input[i] = sin(0.15*pi*i) + sin(0.32*pi*i) + sin(0.4*pi*i);
    		input_fixed[i] =  ((short)((input[i]) * (1 << 8)));
    		//printf("Input fixed data = %hx\r\n", input_fixed[i]);
    }

    /*xil_printf("------------------------------\n\r");
    xil_printf("Input data initialization done\r\n");
    xil_printf("------------------------------\n\r");*/
    /***************************************************************************
    Generating Fixed point Coefficients for 63-tap FIR filtering
    ***************************************************************************/
    int N=63;
    int alpha=(N-1)/2;

    float arr1[N];
    float arr2[N];
    float arr[N];
    float wind[N];
    float h[N];
    short h_fixed[N];
    float w[2];
    /***************************************************************************
    Normalized Cut-Off frequency for a band pass FIR filter
    ***************************************************************************/
    w[0]=0.39;
    w[1]=0.41;
    int k;
    for(i=-alpha;i<=alpha;i++){
      if(i!=0){
                  arr1[i+alpha]=sin(w[0]*pi*i)/(pi*i);
                  arr2[i+alpha]=sin(w[1]*pi*i)/(pi*i);
      }else{
                  arr1[i+alpha]=w[0];
                  arr2[i+alpha]=w[1];
      }
    }

     for(j=0;j<N;j++){
             arr[j]=arr2[j]-arr1[j];
     }
 	/***************************************************************************
     Using Hamming window to generate filter coefficients
     ***************************************************************************/
     for(k=0;k<N;k++){
             wind[k]=(54 - 46*cos(2*pi*k/(N-1)))/100;
     }
     float y1=0,y2=0,y=0.0;
     float val=(w[0]+w[1])/4;
     for(i=0;i<N;i++){
         h[i]=arr[i]*wind[i];
         y1+=cos(2*pi*i*val)*h[i];
         y2+=sin(-2*pi*i*val)*h[i];
     }
     y=(float)sqrt(y1*y1+y2*y2);
     for(i=0;i<N;i++){
         h[i]=(h[i]/y);
         h_fixed[i] = ((short)((h[i]) * (1 << 15)));
         //printf("coeff data = %hx\r\n", h_fixed[i]);
     }
     /***********************************************************
     Applying Fixed point FIR filtering
     ************************************************************/
     int acc; // accumulator for MACs
     short inputp[N]; // pointer to input samples
     int n;
     for ( k = 0; k < N; k++ ) {
      	inputp[k]=0;
     }

     short output_fixed[SAMPLES];
     // apply the filter to each input sample
     init_timer(timer_ctrl, timer_counter_l, timer_counter_h);
     start_timer(timer_ctrl);
     for ( n = 0; n < 1000; n++ ) {
    	 inputp[0] = input_fixed[n];
    	 acc = 1 << 14;
    	 for ( k = 0; k < N; k++ ) {
    		 acc += (int)(h_fixed[k]) * (int)(inputp[k]);
    	 }
    	 for ( k = N; k > 0; k-- ) {
    		 inputp[k]=inputp[k-1];
    	 }
         // saturate the result
     	 if ( acc > 0x3fffffff ) {
     		 acc = 0x3fffffff;
     	 } else if ( acc < -0x40000000 ) {
     		 acc = -0x40000000;
     	 }
     		 // convert from Q30 to Q15
     	 output_fixed[n] = (short)(acc >> 15);
     	 }

     stop_timer(timer_ctrl);
     xil_printf("Execution time %d us in SW for fixed point FIR filtering\n\r", (*timer_counter_l)/333);

     cleanup_platform();
     return 0;
}
