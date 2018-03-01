void state_mov(int *nowState,int *nextState,int action){
    int noiseAction=5;
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

    if( rand()%100<(noise*100) ){
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

int greed_selete(double* Q_table){
    double temp;
    int i,setAction=0,count1=0,eq[4]={0};
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

int e_greed_selete(double* Q_table){
    double temp;
    int i,setAction=0,count1=0,eq[actNum]={0};
    temp=Q_table[0];
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

double updateQ_table(double nowQ,double* nextQ,double r){
    nowQ = nowQ + arc*(r + gama*nextQ[greed_selete(nextQ)]-nowQ);
    return nowQ;
}

