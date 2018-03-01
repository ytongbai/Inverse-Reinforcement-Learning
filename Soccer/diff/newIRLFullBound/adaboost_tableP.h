float alpha(float error_rate){
    float a=0;
    a=log((1-error_rate)/error_rate)/2;
    return a;
}

float weight_change(int *correct,float *ada_weight,float a){
    int i;
    float z=0;
    for(i=0;i<feature_num+1;i++){
        if(correct[i]==0){
            ada_weight[i]=ada_weight[i]*exp(a);
        }else{
            ada_weight[i]=ada_weight[i]*exp(-a);
        }
        z=z+ada_weight[i];
    }
    for(i=0;i<feature_num+1;i++){
        ada_weight[i]=ada_weight[i]/z;
    }
	robot_console_printf("z=%f",z);
    return 0;
}

float classify(float *m_exp,float *m_other,float *ada_weight){
    int i;
    int correct[feature_num+1]={0};
    float error_rate=0,a;
    for(i=0;i<feature_num+1;i++){
        if(m_exp[i]==m_other[i]){
            correct[i]=1;
        }else{
            error_rate=error_rate+ada_weight[i];
        }
    }
    a=alpha(error_rate);
    //printf("\nerror=%f\n",error_rate);
    weight_change(correct,ada_weight,a);
	robot_console_printf(",a=%f\n",a);
    return error_rate;
}
