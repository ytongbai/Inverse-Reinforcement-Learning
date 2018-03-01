#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

int greed_selete(float* Q_table){
    float temp;
    int i,setAction=0,count1=0,eq[actNum]={0};
    temp = Q_table[0];
    for(i=1;i<actNum;i++){
        if(Q_table[i] > temp){
            temp = Q_table[i];
            setAction = i;
        }
    }

    for(i=0;i<actNum;i++){
        if(temp == Q_table[i]){
            eq[count1]=i;
            count1++;
        }
    }
    if(count1 > 1){
        setAction=eq[rand()%(count1)];
    }
    return setAction;
}

int e_greed_selete(float* Q_table){
    float temp;
    int i,setAction=0,count1=0,eq[actNum]={0};
    temp = Q_table[0];
    if(rand()%10 < exp_rate*10){
        for(i=1;i<actNum;i++){
            if(Q_table[i] > temp){
                temp = Q_table[i];
                setAction = i;
            }
        }
		//robot_console_printf("A_Q=[");
        for(i=0;i<actNum;i++){
            if(temp == Q_table[i]){
                eq[count1]=i;
                count1++;
            }
			//robot_console_printf("%f,",Q_table[i]);
        }
		//robot_console_printf("]");
        if(count1 > 1){
            setAction=eq[rand()%(count1)];
        }
    }else{
        setAction=rand()%(actNum);
    }
    return setAction;
}

float updateQ_table(float nowQ,float* nextQ,float r){
	//robot_console_printf("Qupdate[%f,%f,%f]",nowQ,nextQ[greed_selete(nextQ)],r);
    nowQ = nowQ + arc*(r + gama*nextQ[greed_selete(nextQ)]-nowQ);
	//robot_console_printf("Q2update[%f]\n",nowQ);
    return nowQ;
}

