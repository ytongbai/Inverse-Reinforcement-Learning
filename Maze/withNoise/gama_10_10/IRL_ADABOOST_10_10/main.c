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

int main(void){
    srand(time(NULL));
    int i,j,k;
    double avgItr=0;
    double mu_e[feature_num+1]={0};
    double mu_bar1[feature_num+1]={0};
    double omg[feature_num+1]={0};
    e_exp_feature(mu_e);
    system("PAUSE");
    for(i=0;i<trail;i++){
        printf("%d",i);

        for(j=0;j<row;j++){
            for(k=0;k<col;k++){
                policy[i][j]=0;
            }
        }

        for(j=0;j<feature_num+1;j++){
            mu_bar1[j]=0;
            omg[j]=0;
        }


        rl(omg);
        exp_feature(mu_bar1);

        avgItr=avgItr+recursive(mu_e,mu_bar1,omg);
        printf("this time number of itr%f\n",avgItr);
    }
    avgItr=avgItr/(double)trail;
    printf("average number of iteration = %f\n",avgItr);

    return 0;
}

/////找尋state對應的feature值
void feature(int *state,int *ft){
    int i;

    for(i=0 ; i<(feature_num+1) ; i++){
        ft[i]=0;
    }

    /////出界
    if((state[0]+1)<1 || (state[0]+1)>= (row+1) || (state[1]+1)<1 || (state[1]+1)>=(col+1)){
        ft[feature_num]=1;
    }else{
        /////不同state的feature
        ft[state[0]*col+state[1]]=1;
    }

}

/////計算feature expectation
void exp_feature(double *mu){
    int now_state[2]={ptStartS1,ptStartS2};
    int now_feature[feature_num+1];
    int cter=1;
    int i;
    while((now_state[0]!=ptEndS1||now_state[1]!=ptEndS2) && cter<50 ){
        /////policy選擇action並做移動
        if( policy[now_state[0]][now_state[1]] < 2 ){
            now_state[0]=now_state[0]+2*policy[now_state[0]][now_state[1]]-1;
        }else{
            now_state[1]=now_state[1]+2*(policy[now_state[0]][now_state[1]]-2)-1;
        }
        feature(now_state,now_feature);

        for(i=0 ; i<(feature_num+1) ;i++){
            mu[i]=pow(gama,cter)*now_feature[i]+mu[i];
        }

        if(now_feature[feature_num] == 1){
            break;
        }
        cter++;
    }
}

void e_exp_feature(double *mu){
    int now_state[2]={ptStartS1,ptStartS2};
    int now_feature[feature_num+1];
    int numStep=1;
    int i;
    int ctrAction=0;
    //   *******************專家動作 ************************ //
    int actionSave[50]={3,3,3,3,0, 0,0,3,3,1, 1,3,3,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 3,3,3,3,3, 3,3,3,3,3};
    while(now_state[0]!=ptEndS1||now_state[1]!=ptEndS2 && numStep<50 ){
    printf("state[%d %d]\n",now_state[0],now_state[1]);
        /////policy選擇action並做移動
        if( actionSave[ctrAction]<2 ){
            now_state[0]=now_state[0]+2*actionSave[ctrAction]-1;
        }else{
            now_state[1]=now_state[1]+2*(actionSave[ctrAction]-2)-1;
        }
        feature(now_state,now_feature);
        ctrAction++;
        for( i=0;i<(feature_num+1);i++ ){
            mu[i]=pow(gama,numStep)*now_feature[i]+mu[i];
        }
        printf("%f\n",pow(gama,numStep));
        if(now_feature[feature_num] == 1){
            break;
        }
        numStep++;
    }
}

int recursive(double *mu_e,double *mu_bar1,double *omg){
    int i;
    int ctrItr=1;
    double t=0;
    /////init ada_weight
    double temp=0;
    double ada_weight[feature_num+1];
    temp=1/(double)(feature_num+1);
    for(i=0;i<feature_num+1;i++){
        ada_weight[i]=temp;
    }
    for(i=0;i<feature_num+1;i++){
        omg[i]=omg[i]*ada_weight[i];
    }

    while(1){

        printf("ctrItr=%d\n",ctrItr);

        t=0;
        for(i=0;i<feature_num+1;i++){
            t=t+pow(mu_e[i]-mu_bar1[i],2);
        }
        t=pow(t,0.5);
        if(t<e){
            printf("\nThe program is finished.\n");
            break;
        }

        classify(mu_e,mu_bar1,ada_weight);

        for(i=0;i<feature_num+1;i++){
            omg[i]=omg[i]+(mu_e[i]-mu_bar1[i])/t*ada_weight[i];
        }
        rl(omg);

        for(i=0;i<feature_num+1;i++){
            mu_bar1[i]=0;
        }
        exp_feature(mu_bar1);
        ctrItr++;

        if(( ctrItr>maxItr-1 )){
            break;
        }
    }
    return ctrItr;
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
            if(nextState[0] < 0 || nextState[1] < 0 || nextState[0] >= row || nextState[1] >= col){
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
    policyPrintf();
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

int same_trajectory(){
    int i,state[2]={ptStartS1,ptStartS2};
    int ctrStep=1;
    int trajy[row*col]={0};
    int flagT=1;
    static int bTrajy[row*col]={0};

    // ************* 紀錄當前policy學出來的路徑 ************** //
    while( ((state[0] != ptEndS1) || (state[1] != ptEndS2)) && ctrStep<maxStep ){
        trajy[state[0]*row+state[1]]=ctrStep;
        //printf("state[%d %d]",state[0],state[1]);
        if( policy[state[0]][state[1]] < 2 ){
            state[0]=state[0]+2*policy[state[0]][state[1]]-1;
        }else{
            state[1]=state[1]+2*(policy[state[0]][state[1]]-2)-1;
        }
        // ********** 撞牆 *********** //
        if( (state[0]<0) || (state[0]>=row) || (state[1]<0) || (state[1]>=row) ){
            //printf("wall\n");
            break;
        }
        if(trajy[state[0]*row+state[1]]>0){
            //printf("repeat\n");
            break;
        }
        ctrStep++;
    }
    // ******************* (1)與過去的路徑做比較 (2)處存當前路徑 ***************** //

    for(i=0;i<feature_num;i++){
        if(trajy[i]!=bTrajy[i]){
            flagT=0;
            bTrajy[i]=trajy[i];
        }
    }
    return flagT;
}

void policyPrintf(){
    printf("\n");
    int i,j;
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
