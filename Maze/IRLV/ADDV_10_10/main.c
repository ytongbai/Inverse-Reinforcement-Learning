//////recursive 內為何是mu_e-mu_bar1 而不mu_e-mu_bar2

/////問題1.

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <time.h>
#include "Q_parameter.h"
#include "Q_grid.h"
#include "adaboost.h"

#define e 0.0000001

int policy[row][col]={{0}};
int ctrFail=0;
int main(void){
    int i,j,k;
    double omg[row*col+1]={0};
    srand(time(NULL));

	FILE *fp = fopen("Omg.txt","r");
    for(i=0;i<row*col+1;i++){
        fscanf(fp,"%lf",&omg[i]);
    }
    fclose(fp);
	/*
    FILE *fpV = fopen("V.txt","w");
    fclose(fpV);
	*/
    for(i=0;i<trail;i++){
        rl(omg);
        policy_printf();
	}
    //policy_printf();
    return 0;
}

void rl(double *omg){
    double Q_table[row][col][actNum]={{{0}}},reward;
    int startState[2]={ptStartS1,ptStartS2},nowState[2],nextState[2];
    int action,countStep[episode]={0};
    int i,j,k;

    srand(time(NULL));


    /////走到次數
    for(k=0;k<episode;k++){
        nowState[0]=startState[0];
        nowState[1]=startState[1];


        /////起點走到終點
        while(!(nowState[0]==ptEndS1&&nowState[1]==ptEndS2) && maxStep > countStep[k]){

            action=e_greed_selete(Q_table[nowState[0]][nowState[1]]);
            state_mov(nowState,nextState,action);

            /////reward
            if(nextState[0] < 0 || nextState[1] < 0 || nextState[0] >= row || nextState[1] >= col || find_state(nextState) < 0){
                nextState[0] = nowState[0];
                nextState[1] = nowState[1];

                reward=omg[feature_num];
                Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);

            }else{
                reward = omg[nextState[0]*col+nextState[1]];
                Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);
            }

            /////update Q_table
            //Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);

            nowState[0]=nextState[0];
            nowState[1]=nextState[1];

            countStep[k]++;
        }
		for(i=0;i<row;i++){
	        for(j=0;j<col;j++){
	            action=greed_selete(Q_table[i][j]);
	            switch(action){
	                case 0:
	                    policy[i][j]=0;
	                    break;
	                case 1:
	                    policy[i][j]=1;
	                    break;
	                case 2:
	                    policy[i][j]=2;
	                    break;
	                default:
	                    policy[i][j]=3;
	                    break;

	            }
	        }
	    }
		addV(omg);
    }


/*
    printf("\n");
    for(i=0;i<row;i++){
        for(j=0;j<row;j++){
            switch(policy[i][j]){
                case 0:
                    printf("^ ");
                    break;
                case 1:
                    printf("V ");
                    break;
                case 2:
                    printf("< ");
                    break;
                default:
                    printf("> ");
                    break;
            }
        }
        printf("\n");
    }*/
}

void vector_print(double *const source,int num,char *name){
    int i;
    printf("\n%s=[\n",name);
    for(i=0;i<num;i++){
        printf("%lf ",source[i]);
        if(i%row == (row-1)){
            printf("\n");
        }
    }
    printf("]");
}
/*
void vector_fprint(double *const source,FILE *ft,int num,char *name){
    int i;
    fprintf(ft,"\n%s=[",name);
    for(i=0;i<num;i++){
        fprintf(ft,"%f ",source[i]);
    }
    fprintf(ft,"];");
}
*/
void policy_printf(){
    int i,j;
    printf("\n");
    for(i=0;i<row;i++){
        for(j=0;j<row;j++){
            switch(policy[i][j]){
                case 0:
                    printf("^ ");
                    break;
                case 1:
                    printf("V ");
                    break;
                case 2:
                    printf("< ");
                    break;
                default:
                    printf("> ");
                    break;
            }
        }
        printf("\n");
    }
}
void addV(double *omg){
    int now_state[2]={0},ctrStep=0;
    double V=0;
    now_state[0]=ptStartS1;
    now_state[1]=ptStartS2;
    while(ctrStep<maxStep && (now_state[0]!=ptEndS1 || now_state[1]!=ptEndS2)){

        if( policy[now_state[0]][now_state[1]] < 2 ){
            now_state[0]=now_state[0]+2*policy[now_state[0]][now_state[1]]-1;
        }else{
            now_state[1]=now_state[1]+2*(policy[now_state[0]][now_state[1]]-2)-1;
        }

        ctrStep++;
        if(now_state[0] < 0 || now_state[1] < 0 || now_state[0] >= row || now_state[1] >= col){
            V=V+omg[feature_num]*pow(gama,ctrStep);
            break;
        }else{
            V=V+omg[now_state[0]*col+now_state[1]]*pow(gama,ctrStep);
        }
    }
    printf("V=%f\n",V);
    FILE *fpV = fopen("V.txt","a");
    fprintf(fpV,"%lf\n",V);
    fclose(fpV);

}
