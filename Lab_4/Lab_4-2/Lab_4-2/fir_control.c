// function: From implement fir to write input and read output
#include "fir.h"

void __attribute__ ( ( section ( ".mprjram" ) ) ) initfir() {
	//initial your fir
	for(int k = 0; k < N; k++){
	    inputbuffer[k] = inputsignal[k];
	    outputsignal[k] = 0;
	}
}

int* __attribute__ ( ( section ( ".mprjram" ) ) ) fir(){
	initfir();
	//write down your fir	
	for(int i = 0; i < N; i++){
	    for(int j = 0; j <= i; j++){
	        outputsignal[i] += taps[j] * inputbuffer[i-j];
	    }
	}
	return outputsignal;
}
		
