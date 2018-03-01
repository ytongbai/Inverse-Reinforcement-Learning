//////Apprenticeship Learning via Inverse Reinforcement Learning

#define e 0.0000001

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <time.h>



#include "Q_parameter.h"
int policy[row][col]={{0}};
int policy_e[row][col]={{0}};
#include "Q_grid.h"


int main(void){
    int i,j;
    int ctrTrail=0;
    int avgItr=0,sum;
    srand(time(NULL));

    double mu_e[feature_num+1]={0};
    e_exp_feature(mu_e);

    for(ctrTrail=0;ctrTrail<trail;ctrTrail++){
        printf("ctrTrail=%d",ctrTrail);
        for(i=0;i<row;i++){
            for(j=0;j<col;j++){
                policy[i][j]=rand()%4;
            }
        }

        double mu_bar1[feature_num+1]={0};
        exp_feature(mu_bar1);

        double mu_bar2[feature_num+1]={0};
        double mu_2[feature_num+1]={0};

        for(i=0;i<feature_num+1;i++){
            mu_bar2[i]=mu_bar1[i];
        }

        double omg[feature_num+1]={0};
        sum=recursive(mu_e,mu_bar1,mu_2,mu_bar2,omg);
        avgItr=avgItr+sum;
        printf("this time iteration =%d\n",avgItr);
    }
    avgItr=avgItr/trail;
    printf("\naverage of iteration = %d ",avgItr);

    system("PAUSE");
    return 0;
}


