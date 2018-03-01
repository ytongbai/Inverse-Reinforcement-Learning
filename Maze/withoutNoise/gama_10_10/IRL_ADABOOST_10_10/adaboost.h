// calculate alpha
double alpha(double error_rate){
    double a=0;
    a=log((1-error_rate)/error_rate);
    return a;
}
// update weight
double weight_change(int *correct,double *ada_weight,double a){
    int i;
    double z=0;
    for(i=0;i<feature_num+1;i++){
        if(correct[i]==0){
            ada_weight[i]=ada_weight[i]*exp(a);
        }else{
            //ada_weight[i]=ada_weight[i]*exp(-a);
        }
        z=z+ada_weight[i];
    }
    for(i=0;i<feature_num+1;i++){
        ada_weight[i]=ada_weight[i]/z;
    }
    return 0;
}
// calculate error_rate
double classify(double *m_exp,double *m_other,double *ada_weight){
    int i;
    int correct[feature_num+1]={0};
    double error_rate=0,a;

    for(i=0;i<feature_num+1;i++){
        if(m_exp[i]==m_other[i]){
            correct[i]=1;
        }else{
            error_rate=error_rate+ada_weight[i];
        }
    }
    a=alpha(error_rate);

    weight_change(correct,ada_weight,a);

    return error_rate;
}
