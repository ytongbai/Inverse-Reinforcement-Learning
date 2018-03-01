
void state_mov(int *nowState,int *nextState,int action){
    int noiseAction=5;
    // take normal action
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
    // noise happen
    if( rand()%100<(noise*100) ){
        // choose which action
        //// (1)Don't move (2) action (3) action + noise action(left) (4) action +noise action(right)
        noiseAction=rand()%4;
        switch(noiseAction){
            //一般
            case 0:
                break;
            //原來的位置
            case 1:
                nextState[0]=nowState[0];
                nextState[1]=nowState[1];
                break;
            case 2:
                if(action<2){
                    nextState[1]=nowState[1]-1;
                }else{
                    nextState[0] = nowState[0]-1;
                }
                break;
            case 3:
                if(action<2){
                    nextState[1]=nowState[1]+1;
                }else{
                    nextState[0] = nowState[0]+1;
                }
                break;
        }
    }

}
// greed to take action
int greed_selete(double* Q_table){
    double temp;
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
// epsilon-greed to take action
int e_greed_selete(double* Q_table){
    double temp;
    int i,setAction=0,count1=0,eq[actNum]={0};
    temp = Q_table[0];
    if(rand()%10 < exp_rate*10){
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
    }else{
        setAction=rand()%(actNum);
    }
    return setAction;
}
// update Q
double updateQ_table(double nowQ,double* nextQ,double r){
    nowQ = nowQ + arc*(r + gama*nextQ[greed_selete(nextQ)]-nowQ);
    return nowQ;
}

// 找尋state對應的feature值
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
    while( (now_state[0]!=ptEndS1||now_state[1]!=ptEndS2) && cter<50 ){
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
    while( ((now_state[0]!=ptEndS1) || (now_state[1]!=ptEndS2)) && (numStep<50) ){

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

        if(now_feature[feature_num] == 1){
            break;
        }
        numStep++;
    }

}
// orthogonal projection to find the mu_bar2
void maxmin_projection(double *mu_e,double *mu_bar1,double *mu_2,double *mu_bar2){

    double par_temp1=0;
    double par_temp2=0;
    int i;
    for(i=0;i<feature_num+1;i++){
        par_temp1=par_temp1+(mu_2[i]-mu_bar1[i])*(mu_e[i]-mu_bar1[i]);
        par_temp2=par_temp2+(mu_2[i]-mu_bar1[i])*(mu_2[i]-mu_bar1[i]);
    }
    for(i=0;i<feature_num+1;i++){
        mu_bar2[i]=mu_bar1[i]+par_temp1/par_temp2*(mu_2[i]-mu_bar1[i]);
    }
}

void rl(double *omg){
    double Q_table[row][col][actNum]={{{0}}},reward;
    int nowState[2],nextState[2];
    int action,countStep[episode]={0};
    int i,j,k;
    srand(time(NULL));

    FILE *ft=fopen("1.txt","w");
    /////走到次數
    for(k=0;k<episode;k++){
        nowState[0]=ptStartS1;
        nowState[1]=ptStartS2;


        /////起點走到終點
        while(!(nowState[0]==ptEndS1&&nowState[1]==ptEndS2) && maxStep > countStep[k]){
            fprintf(ft,"[%d %d] ",nowState[0],nowState[1]);
            action=e_greed_selete(Q_table[nowState[0]][nowState[1]]);
            state_mov(nowState,nextState,action);

            /////reward
            //if(nextState[0] < 0 || nextState[1] < 0 || nextState[0] >= row || nextState[1] >= col || find_state(nextState) < 0){
            if( nextState[0] < 0 || nextState[1] < 0 || nextState[0] >= row || nextState[1] >= col ){
                nextState[0] = nowState[0];
                nextState[1] = nowState[1];
                //Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);
                //fprintf(ft,"[%d %d] break",nextState[0],nextState[1]);
                reward = omg[feature_num];
                Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);
                //Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_null,reward);
                //break;
            }else{
                reward = omg[nextState[0]*col+nextState[1]];
            }
            //printf("\nnow=(%d,%d),act=%d,next=(%d,%d)",nowState[0],nowState[1],action,nextState[0],nextState[1]);
            /////update Q_table
            Q_table[nowState[0]][nowState[1]][action]=updateQ_table(Q_table[nowState[0]][nowState[1]][action],Q_table[nextState[0]][nextState[1]],reward);

            nowState[0]=nextState[0];
            nowState[1]=nextState[1];
            //printf("nowState=(%d,%d)",nowState[0],nowState[1]);

            countStep[k]++;
        }
        fprintf(ft,"\ns");
        //fprintf(fp1,"%d ",countStep[p][k]);
    }
    //fprintf(fp1,"\n");
    //printf("\ncountStep=%d",countStep[k-1]);
    fclose(ft);

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
            //printf("%d ",trajy[i]);
        }
    }
    //printf("f%df",flagT);
    return flagT;
}

void vector_print(double *const source,int num,char *name){
    int i;
    printf("\n%s=[",name);
    for(i=0;i<num;i++){
        printf("%f ",source[i]);
    }
    printf("]");
}

int recursive(double *mu_e,double *mu_bar1,double *mu_2,double *mu_bar2,double *omg){
    int i;
    int ctrItr=1;
    double t=0;

    while(1){
        t=0;
        printf("ctrItr=%d\n",ctrItr);
        if(ctrItr!=1){
            maxmin_projection(mu_e,mu_bar1,mu_2,mu_bar2);
        }
        for(i=0;i<feature_num+1;i++){
            omg[i]=mu_e[i]-mu_bar2[i];
        }

        for(i=0;i<feature_num+1;i++){
            t=t+pow(omg[i],2);
        }
        t=pow(t,0.5);

        /*
        FILE *fpT=fopen("T.txt","a");
        fprintf(fpT,"%f",t);
        fclose(fpT);
        */
        if( t<e ){
            printf("\nThe program is finished.num=%d",ctrItr);
            break;
        }else{
            rl(omg);
            for(i=0;i<feature_num+1;i++){
                mu_bar1[i]=mu_bar2[i];
                mu_2[i]=0;
            }
            exp_feature(mu_2);
            ctrItr++;
        }

        if(ctrItr>maxItr-1){break;}
    }

    return ctrItr;
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

