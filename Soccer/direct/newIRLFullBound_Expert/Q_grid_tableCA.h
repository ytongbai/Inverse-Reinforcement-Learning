#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Q_parameter_tableCA.h"

int find_state(int* state){
    int state_table1[row][col]={0};
    /////wall
    state_table1[1][3]=-1;
    state_table1[2][3]=-1;
    state_table1[3][3]=-1;

    state_table1[3][2]=-1;
    state_table1[3][1]=-1;
    /////goal
    state_table1[0][4]=1;
    return state_table1[state[0]][state[1]];
}

void state_mov(int *nowState,int *nextState,int action){

    switch(action) {
        // mov up
        case 0:
            nextState[1] = nowState[1];
            nextState[0] = nowState[0]-1;
            break;
        //mov down
        case 1:
            nextState[1] = nowState[1];
            nextState[0] = nowState[0]+1;
            break;
        //mov left
        case 2:
            nextState[0] = nowState[0];
            nextState[1] = nowState[1]-1;
            break;
        //mov right
        default:
            nextState[0] = nowState[0];
            nextState[1] = nowState[1]+1;
    }

}

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

int softmax_selete(float* Q_table,int p){
    int i;
    float temp[4],total,prob,t;
    t=l;
    t=t*pow(0.9,p);
    if(t<1){
        t=1;
    }
    total=exp(Q_table[0]/t)+exp(Q_table[1]/t)+exp(Q_table[2]/t)+exp(Q_table[3]/t);
    for(i=0;i<=3;i++){
        temp[i]=exp(Q_table[i]/t);
    }
    prob=(rand()%100+1)*(temp[0]+temp[1]+temp[2]+temp[3])/100;
    if(temp[0] < prob){
        if(temp[0]+temp[1] < prob){
            if(temp[0]+temp[1]+temp[2] < prob){
                return 3;
            }else{
                return 2;
            }
        }else{
            return 1;
        }
    }else{
        return 0;
    }
}

float updateQ_table(float nowQ,float* nextQ,float r){
	//robot_console_printf("Qupdate[%f,%f,%f]",nowQ,nextQ[greed_selete(nextQ)],r);
    nowQ = nowQ + arc*(r + gama*nextQ[greed_selete(nextQ)]-nowQ);
	//robot_console_printf("Q2update[%f]\n",nowQ);
    return nowQ;
}

