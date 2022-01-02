/** \file algorithm.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: algorithm.cpp
* Description: This module calculates the heart rate/SpO2 level*/
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include <stdlib.h>

#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "algorithm.h"

void maxim_heart_rate_and_oxygen_saturation(uint16_t *ir_buffer,  int32_t ir_buffer_length, uint16_t *red_buffer, int32_t *spo2, int8_t *spo2_valid, 
                              int32_t *heart_rate, int8_t  *hr_valid)
{
    uint32_t ir_mean ,only_once ;
    int32_t k ,i_ratio_count;
    int32_t i, s, m, middle_idx;
    int32_t th1, npks,c_min;
    int32_t ir_valley_locs[15] ;
    int32_t peak_interval_sum;

    int32_t y_ac, x_ac;
    int32_t spo2_calc; 
    int32_t y_dc_max, x_dc_max; 
    int32_t y_dc_max_idx, x_dc_max_idx; 
    int32_t ratio[5],ratio_average; 
    int32_t nume,  denom ;

    // remove DC of ir signal,invert and put in x    
    ir_mean = 0; 
    for (k=0 ; k<ir_buffer_length ; k++ ) ir_mean += ir_buffer[k] ;
    ir_mean =ir_mean/ir_buffer_length ;

    for (k=0 ; k<ir_buffer_length ; k++ )  {
	x[k] = -1*(ir_buffer[k] - ir_mean) ;
    }

    //find threshold for peak based on average distance from 0
    th1=0;
    for (k=0; k<BUFFER_SIZE; k++){
    	th1+=((x[k]>0)? x[k] : ((int32_t)0-x[k])) ;
    }

    th1=th1/(BUFFER_SIZE);

    if( th1<30) th1=30; // min allowed
    if( th1>60) th1=60; // max allowed

    for ( k=0 ; k<15;k++) ir_valley_locs[k]=0;
    //find peaks (valleys of original
    maxim_find_peaks( ir_valley_locs, &npks, x, BUFFER_SIZE, th1, 4, 10 );

    //Find average interval between peaks to find HR
    peak_interval_sum =0;
    if (npks>=2){
        for (k=1; k<npks; k++)
            peak_interval_sum += (ir_valley_locs[k]-ir_valley_locs[k -1]);
        peak_interval_sum=peak_interval_sum/(npks-1);
        *heart_rate=(int32_t)((FS*60)/peak_interval_sum);    // beats per minute
        *hr_valid  = 1;
    }
    else  {
        *heart_rate = -999;
        *hr_valid  = 0;
    }

    //cast from uint16_t to int16_t
    for (k=0 ; k<ir_buffer_length ; k++ )  {
        x[k] =  (ir_buffer[k]>>1) ;
        y[k] =  (red_buffer[k]>>1) ;
    }

    //Find the AC/DC max values (between valleys) of IR and red to get SpO2 ratio
    ratio_average =0;
    i_ratio_count =0;
    for(k=0; k< 5; k++) ratio[k]=0;

    //validating locations
    for (k=0; k< npks; k++){
        if (ir_valley_locs[k] > BUFFER_SIZE ){
            *spo2 =  -999 ;
            *spo2_valid  = 0;
            return;
        }
    }

    //find max between two valley locations
    // and use ratio betwen AC component of Ir & Red and DC component of Ir & Red for SpO2
    for (k=0; k< npks-1; k++){
        y_dc_max= -16777216 ;
        x_dc_max= - 16777216;
        if (ir_valley_locs[k+1]- ir_valley_locs[k] >3){
	    for (i= ir_valley_locs[k]; i< ir_valley_locs[k+1]; i++){
                if (x[i]> x_dc_max) {x_dc_max =x[i]; x_dc_max_idx=i;}
                if (y[i]> y_dc_max) {y_dc_max =y[i]; y_dc_max_idx=i;}
            }
            y_ac= (y[ir_valley_locs[k+1]] - y[ir_valley_locs[k] ] )*(y_dc_max_idx - ir_valley_locs[k]); //Using linear interpolation
            y_ac=  y[ir_valley_locs[k]] + y_ac/ (ir_valley_locs[k+1] - ir_valley_locs[k])  ;
            y_ac=  y[y_dc_max_idx] - y_ac;    // subracting linear DC compoenents from raw

            x_ac= (x[ir_valley_locs[k+1]] - x[ir_valley_locs[k] ] )*(x_dc_max_idx - ir_valley_locs[k]); // ir
            x_ac=  x[ir_valley_locs[k]] + x_ac/ (ir_valley_locs[k+1] - ir_valley_locs[k]);
            x_ac=  x[y_dc_max_idx] - x_ac;

            nume=( y_ac *x_dc_max)>>7 ; //converting to ratio
            denom= ( x_ac *y_dc_max)>>7 ;
            if (denom>0  && i_ratio_count <5 &&  nume != 0)
            {
                ratio[i_ratio_count]= (nume*100)/denom ; //formula is ( y_ac *x_dc_max) / ( x_ac *y_dc_max) ;
                i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(ratio, i_ratio_count);
    middle_idx= i_ratio_count/2;

    if (middle_idx >1)
        ratio_average =( ratio[middle_idx-1] +ratio[middle_idx])/2; // use median ratio
    else
        ratio_average = ratio[middle_idx ];

    if( ratio_average>2 && ratio_average <184){
        spo2_calc= spo2_table[ratio_average] ;    //use SpO2 lookup table
        *spo2 = spo2_calc ;
        *spo2_valid  = 1;
    }
    else{
        *spo2 =  -999 ; //signal ratio is out of range
        *spo2_valid  = 0;
    }
}


void maxim_find_peaks(int32_t *locs, int32_t *npks, int32_t *x, int32_t size, int32_t min_height, int32_t min_distance, int32_t max_num)
{
    maxim_peaks_above_min_height( locs, npks, x, size, min_height );	//peaks above the threshold height
    maxim_remove_close_peaks( locs, npks, x, min_distance );		//remove peaks closer than min_distance
    *npks = min( *npks, max_num );					//remove excess peaks
}

void maxim_peaks_above_min_height(int32_t *locs, int32_t *npks, int32_t  *x, int32_t size, int32_t min_height)
{
    int32_t i = 1, width;
    *npks = 0;
   /*	Original code finding single and flat peaks	*/
   /* while (i < size-1){
        if (x[i] > min_height && x[i] > x[i-1]){            // find left edge of potential peaks
            width = 1;
            while (i+width < size && x[i] == x[i+width])    // find flat peaks
                width++;
            if (x[i] > x[i+width] && (*npks) < 15 ){        // find right edge of peaks
                locs[(*npks)++] = i;
                // for flat peaks, peak location is left edge
                i += width+1;
            }
            else
                i += width;
        }
        else
            i++;
    }*/

    /*    New code finding single peaks	*/
    /*for (i=1; i<size-1; i++){
    	if (x[i] > min_height && x[i] > x[i-1] && x[i] > x[i+1]){
    		locs[(*npks)++] = i;
	}
    }*/

    /*	Finds single peaks and waits until there is a trough before finding another	*/
    while (i<size-1){
    	if (x[i] > min_height && x[i] > x[i-1] && x[i] > x[i+1]){ //higher than surrounding peaks
		locs[(*npks)++] = i;
		while(i<size-1 && x[i] > x[i+1]){		  //loop until rises again
			i++;
		}
		i++;	//skip one in case it immediately goes back down
	}
	i++;
    }
}


void maxim_remove_close_peaks(int32_t *locs, int32_t *npks, int32_t *x, int32_t min_distance)
{
    int32_t i, j, old_npks, dist;

    //Prioritise large peaks
    maxim_sort_indices_descend( x, locs, *npks );

    for ( i = -1; i < *npks; i++ ){
        old_npks = *npks;
        *npks = i+1;
        for ( j = i+1; j < old_npks; j++ ){
            dist =  locs[j] - ( i == -1 ? -1 : locs[i] ); //check all peaks distance from highest
            if ( dist > min_distance || dist < -min_distance )
                locs[(*npks)++] = locs[j];
        }
    }

    // Sort peaks in order again
    maxim_sort_ascend( locs, *npks );
}

void maxim_sort_ascend(int32_t *x,int32_t size) 
{
    int32_t i, j, temp;

    //insertion sort
    for (i = 1; i < size; i++) {
        temp = x[i];
        for (j = i; j > 0 && temp < x[j-1]; j--)
            x[j] = x[j-1];
        x[j] = temp;
    }
}

void maxim_sort_indices_descend(int32_t *x, int32_t *indx, int32_t size)
{
    int32_t i, j, temp;

    //insertion sort
    for (i = 1; i < size; i++) {
        temp = indx[i];
        for (j = i; j > 0 && x[temp] > x[indx[j-1]]; j--)
            indx[j] = indx[j-1];
        indx[j] = temp;
    }
}
