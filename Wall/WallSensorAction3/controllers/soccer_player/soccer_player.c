// 改寫 mybot的範例程式 和 足球範例程式 //

// 功能
// Qlearning(policy,mu1)->update funciton(ada weight)->IRL(reward)

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <device/robot.h>
#include <device/differential_wheels.h>
#include <device/receiver.h>
#include <device/gps.h>
#include <device/distance_sensor.h> 

// feature_num為總共特徵數量 sNum為總共狀態數量 rNum為距離的狀態切割數量 tNum為角度的狀態切割數量
// actNum為動作的數量
#define feature_num 120+1
#define sNum 120+1
#define rNum 10
#define tNum 12
#define actNum 4
// ***** Q_learning parameter ***** //
// 參數設定
// arc為學習率 gama為折扣率 exp_rate為探索率
#define arc 0.7
#define exp_rate 0.8
#define gama 0.7

#include <Q_func.h>
#include <adaboost_wall.h>
#define ROBOTS 1
#define TIME_STEP 64

#define desireError 0.0001 // 預期與專家間的差距
#define gamaIrl gama
#define numFeature sNum+1

// ***** function ***** //
static void reset(void);
static int run(int);
void Fstate(float *,int *,int *,int); // 位置轉換為狀態
// 接下來三個函式，計算最接近機器人的牆與機器人的夾角與距離
void minSensor(unsigned short *,int *); 
float heron_formula(int,int);
void thetaWall(unsigned short *,float *);
// ***** 變數 ***** //
// ********** 裝置 ********** //
static DeviceTag receiver;
static DeviceTag ds0,ds1,ds2,ds3,ds4,ds5,ds6,ds7,ds8,ds9,ds10,ds11;
// ********** 值 ********** //
static float tableQ[sNum][actNum]={{0}}; // Q table
static float weightAda[sNum+1]={0}; // adaboost中的權重
static float mu1[sNum+1]={0}; // 學出來的特徵期望值
static float mue[sNum+1]={0}; // 專家的特徵期望值
static float omg[sNum+1]={0}; // reward function中的w

// ********** 旗標 ********** //
static int flagFinish=0; // 以學出與專家相同的策略時開啟此旗標

static void reset(void)
{
  robot_console_printf("RobotR reseting..");
  int i,j;
  const char *name;
  name = robot_get_name();
  srand(time(NULL));
  // 紅外線初始化
  ds0=robot_get_device("ds0");
  ds1=robot_get_device("ds1");
  ds2=robot_get_device("ds2");
  ds3=robot_get_device("ds3");
  ds4=robot_get_device("ds4");
  ds5=robot_get_device("ds5");
  ds6=robot_get_device("ds6");
  ds7=robot_get_device("ds7");
  ds8=robot_get_device("ds8");
  ds9=robot_get_device("ds9");
  ds10=robot_get_device("ds10");
  ds11=robot_get_device("ds11");
  
  distance_sensor_enable(ds0, TIME_STEP);
  distance_sensor_enable(ds1, TIME_STEP);
  distance_sensor_enable(ds2, TIME_STEP);
  distance_sensor_enable(ds3, TIME_STEP);
  distance_sensor_enable(ds4, TIME_STEP);
  distance_sensor_enable(ds5, TIME_STEP);
  distance_sensor_enable(ds6, TIME_STEP);
  distance_sensor_enable(ds7, TIME_STEP);
  distance_sensor_enable(ds8, TIME_STEP);
  distance_sensor_enable(ds9, TIME_STEP);
  distance_sensor_enable(ds10, TIME_STEP);
  distance_sensor_enable(ds11, TIME_STEP);
  
  receiver = robot_get_device("receiver");
  receiver_enable(receiver,TIME_STEP);  
  
  
  // *****紀錄的項目 Q_TABLE、Omg、Mu_e、Weight_Ada 、Finish_Flag***** //
  FILE *fp = fopen("Q_table.txt","r");
  if(fp != NULL){
    for(i=0;i<sNum;i++){
      for(j=0;j<actNum;j++){
        fscanf(fp,"%f",&tableQ[i][j]);
      }
    }
    fclose(fp);
  }
  
  FILE *fpOmg=fopen("Omg.txt","r");
  if(fpOmg != NULL){
    for(i=0;i<sNum+1;i++){
      fscanf(fpOmg,"%f",&omg[i]);
    }
    fclose(fpOmg);
  }
  
  FILE *fpMue=fopen("Mue.txt","r");
  if(fpMue != NULL){
    for(i=0;i<sNum+1;i++){
      fscanf(fpMue,"%f",&mue[i]);
    }
    fclose(fpMue);
  }else{
    robot_console_printf("No mu_e");
    system("PAUSE");
  }
  
  FILE *fpWeightAda=fopen("WeightAda.txt","r");
  if(fpWeightAda != NULL){
    for(i=0;i<sNum+1;i++){	
      fscanf(fpWeightAda,"%f",&weightAda[i]);
    }
    fclose(fpWeightAda);
  }else{
    for(i=0;i<sNum+1;i++){	
      weightAda[i]=1/(float)(sNum+1);
    }
  }
	// *****終止旗標設為0***** //
  FILE *fpFinish=fopen("Finish.txt","r");
  if(fpFinish != NULL){
    fscanf(fpFinish,"%d",&flagFinish);
    fclose(fpFinish);
  }
  robot_console_printf("RobotR reset done\n");
}

static int run(int ms)
{
  const float *buffer; // supervisor傳送來的資訊
  
  static int flagMu1One=0; // 紀錄mu_1已經計算結束
  static int flagWallOne=0; // 紀錄此時為撞牆
  static int flagGoalOne=0; // 讓Q_table只記錄一次除非球進
  
  static int ctrWall=0; // 紀錄狀態不變化的次數 當大於某個值判斷為撞牆
  static int ctrInput=0; // 由於剛開始supervisor傳送的值未必正確 因此從第二次傳入的值開始做計算
  static int ctrTrajectory=0; // 計算mu值作為紀錄第幾步用
  static int ctrStep=0; // 紀錄該回合使用的步數
  
  static int bState[2]={0}; // 前一個狀態
  static int bAction=0; // 上次選擇的動作
  static int state[2]; // 當前狀態與與這次選取的動作  
  int action;
  
  unsigned short v[12]; // 各個sensor抓到的距離
  
  // 左右輪子的速度
  int left_speed=0;
  int right_speed=0;
  
  float endQ[actNum]={0};
  float sensorInform[2]={0}; // 最近距離和角度
  float tempQ;
  
  float errorNow=0; //專家與學習出來的mu之間的距離
  int i,j,n;
  left_speed = 0;
  right_speed = 0;
  
  v[0]=distance_sensor_get_value(ds0);
  v[1]=distance_sensor_get_value(ds1);
  v[2]=distance_sensor_get_value(ds2);
  v[3]=distance_sensor_get_value(ds3);
  v[4]=distance_sensor_get_value(ds4);
  v[5]=distance_sensor_get_value(ds5);
  v[6]=distance_sensor_get_value(ds6);
  v[7]=distance_sensor_get_value(ds7);
  v[8]=distance_sensor_get_value(ds8);
  v[9]=distance_sensor_get_value(ds9);
  v[10]=distance_sensor_get_value(ds10);
  v[11]=distance_sensor_get_value(ds11);
  
  n = receiver_get_buffer_size(receiver);
  
  if(n  && flagMu1One==0){
    // ***** 將Input轉為State  並計算action ***** //
    buffer = (const float *) receiver_get_buffer(receiver);    
    // initail state and action //
    if(ctrInput == 1){
      bState[0]=8;
      bState[1]=3;
      state[0]=8;
      state[1]=3;
      action=greed_selete(tableQ[state[0]*tNum+state[1]]);
      bAction=action;
    }
    if(ctrInput > 1){
            
      thetaWall(v,sensorInform);      
      Fstate(sensorInform,state,bState,action);
      if( buffer[4]<0.5 ){ // 判斷episode
        action=e_greed_selete(tableQ[state[0]*tNum+state[1]]);
        
        if(buffer[2]<0.5 && buffer[3]<0.5 && flagWallOne != 1){ // 判斷訓練結束
          if(state[0]==bState[0] && state[1]==bState[1]){ // 判斷撞牆
            ctrWall++;
            
            if( ctrWall>600 ){  // 判斷在原地使否待太久 作為是否撞牆的依據
              // 撞牆 更新Q_table
              tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[bState[0]*tNum+bState[1]],omg[sNum]);
              tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;
              
              ctrWall=0;
              bAction=action;
              ctrStep++;
              
            }
            action=bAction; // 若薇相同狀態則執行相同動作
          }else{              
            // 沒有撞牆 更新Q_table
            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[state[0]*tNum+state[1]],omg[state[0]*tNum+state[1]]);
            tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;
            
            ctrWall=0;
            bAction=action;
            bState[0]=state[0];
            bState[1]=state[1];
            ctrStep++;
          }
        }else{
          if(buffer[3]>0.5 && flagGoalOne!=2){
            // 到目標 更新Q_table
            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],endQ,omg[sNum-1]);
            tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;
            // 記錄Q_table.txt
            robot_console_printf("writing the Goal_Q.  ");
            FILE *fp = fopen("Q_table.txt","w");          
            for(i=0;i<sNum;i++){
              for(j=0;j<actNum;j++){
                fprintf(fp,"%f\n",tableQ[i][j]);
              }
            }
            fclose(fp);
            robot_console_printf("success writing the Goal_Q\n");
            flagGoalOne = 2;

          }else if(flagGoalOne==0){
            // 記錄Q_table.txt
            robot_console_printf("writing the Q.  ");
            FILE *fp = fopen("Q_table.txt","w");          
            for(i=0;i<sNum;i++){
              for(j=0;j<actNum;j++){
                fprintf(fp,"%f\n",tableQ[i][j]);
              }
            }
            fclose(fp);
            robot_console_printf("success writing the Q\n");
            flagGoalOne = 1;
          }        
        }
      }else{ // 計算mu值
        action=greed_selete(tableQ[state[0]*tNum+state[1]]);
        if(buffer[2]<0.5 && buffer[3]<0.5 && flagWallOne != 1){
        
          if( state[0]==bState[0] && state[1]==bState[1] ){
            ctrWall++;
            if( ctrWall>600 ){
              bAction=action;
              ctrWall=0;
                
              ctrTrajectory++;
              mu1[bState[0]*tNum+bState[1]]=mu1[bState[0]*tNum+bState[1]]+pow(gamaIrl,ctrTrajectory);
              robot_console_printf("stateMove(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],bState[0]*tNum+bState[1],mu1[bState[0]*tNum+bState[1]]);
              robot_console_printf("tableQ[%d][%d]=%f,norm\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
              ctrTrajectory++;
              mu1[sNum]=mu1[sNum]+pow(gama,ctrTrajectory);            
              robot_console_printf("stateWall  (%d)[wall],mu1[%d]=%f",ctrTrajectory,sNum,mu1[sNum]);
              robot_console_printf("tableQ[wall]=%f,wall\n",tableQ[bState[0]*tNum+bState[1]][bAction]);
              flagWallOne=1;
            }
            action=bAction;
          }else{              
            ctrTrajectory++;
            mu1[bState[0]*tNum+bState[1]]=mu1[bState[0]*tNum+bState[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateMove(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],bState[0]*tNum+bState[1],mu1[bState[0]*tNum+bState[1]]);
            robot_console_printf("tableQ[%d][%d]=%f,norm\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
            ctrWall=0;            
            bAction=action;  
            bState[0]=state[0];
            bState[1]=state[1];
          }
        }else{
          if( flagWallOne!=1 ){
            ctrTrajectory++;
            mu1[state[0]*tNum+state[1]]=mu1[state[0]*tNum+state[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateEnd(%d)[%d %d],mue[%d]=%f\n",ctrTrajectory,state[0],state[1],state[0]*tNum+state[1],mue[state[0]*tNum+state[1]]);
            ctrTrajectory++;
            mu1[sNum-1]=mu1[sNum-1]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateEnd  (%d)[%d],mu1[%d]=%f\n",ctrTrajectory,sNum-1,sNum-1,mu1[sNum-1]);
          }
          //計算誤差值
          for(i=0;i<sNum+1;i++){
            errorNow=errorNow+pow(mue[i]-mu1[i],2);
          }
          errorNow=pow(errorNow,0.5);
          
          robot_console_printf("errorSumNow=%f\n",errorNow);
          if(errorNow > desireError){
            // update adaboost的權重值
            classify(mue,mu1,weightAda);
            
            FILE *fpOmg=fopen("Omg.txt","w");
            for(i=0;i<sNum+1;i++){
              // update omg的值
              omg[i]=omg[i]+(mue[i]-mu1[i])/errorNow*weightAda[i];
              fprintf(fpOmg,"%f\n",omg[i]);
            }
            fclose(fpOmg);
            
            FILE *fpWeightAda=fopen("WeightAda.txt","w");         
            for(i=0;i<sNum+1;i++){
              fprintf(fpWeightAda,"%f\n",weightAda[i]);
            }
            fclose(fpWeightAda);        
          }else{// 已學會 結束
            
            flagFinish=1;
            
            FILE *fpFinish=fopen("Finish.txt","w");
            fprintf(fpFinish,"%d\n",flagFinish);
            fclose(fpFinish);
            robot_console_printf("finish!!");
            system("PAUSE");
          }     
          flagMu1One=1;
          
        }
      }
      
      
      if(buffer[2]<0.5 && buffer[3]<0.5 && flagFinish!=1){ // 球沒移動或時間沒到或是還沒學完成前 針對不同動作控制兩個輪子的轉速
      
        switch(action){        
          case 0:
            if( (int)sensorInform[1]%30>2 && (int)sensorInform[1]%30<10 ){
              left_speed = 10;
              right_speed = -10;
            }else if( (int)sensorInform[1]%30<28 && (int)sensorInform[1]%30>20){
              left_speed = -10;
              right_speed = 10;
            }else{
              left_speed = 10;
              right_speed = 10;
            }           
            break;
          case 1:
            if( (int)sensorInform[1]%30>2 && (int)sensorInform[1]%30<10 ){
              left_speed = 10;
              right_speed = -10;
            }else if( (int)sensorInform[1]%30<28 && (int)sensorInform[1]%30>20){
              left_speed = -10;
              right_speed = 10;
            }else{
              left_speed = -10;
              right_speed = -10;
            }           
            break;
          case 2:
            left_speed = -10;
            right_speed = 10;
            break;
          case 3:
            left_speed = 10;
            right_speed = -10;
            break;
        }        
      }else{
        left_speed = 0;
        right_speed = 0;
      }
      
    }
    ctrInput++; 
  }
  differential_wheels_set_speed(left_speed, right_speed);
 
  return TIME_STEP;           /* step of TIME_STEP ms */
}

int main()
{
    robot_live(reset);
    robot_run(run);             /* never returns */

    return 0;
}


void Fstate(float *sensorInform,int *state,int *bState,int action){
  // ***** radious num 13 ****** //
  float limitR[10]={0.04,0.05,0.07,0.1,0.14,0.19,0.25,0.32,0.4,0.52};
  //float limitR[10]={0.04,0.045,0.085,0.12,0.165,0.22,0.285,0.36,0.46};
  float limitT[12]={0,30,60,90,120,150,180,210,240,270,300,330};
  //robot_console_printf("action=%d,temp=%f,state=%d",action,temp,state[0]);
  switch(action){
    // ****** distance num 10 ***** //
    case 0:
      if( bState[0]>0 ){
        if( sensorInform[0]<limitR[bState[0]-1] ){
          state[0]=bState[0]-1;
        }
      }
      if( bState[0]<9 ){
        if( sensorInform[0]>limitR[bState[0]+1] ){
          state[0]=bState[0]+1;
        }
      }
      break;
    case 1:
      if( bState[0]>0 ){
        if( sensorInform[0]<limitR[bState[0]-1] ){
          state[0]=bState[0]-1;
        }
      }
      if( bState[0]<9 ){
        if( sensorInform[0]>limitR[bState[0]+1] ){
          state[0]=bState[0]+1;
        }
      }
      break;
    // ****** theta num 12 ***** //
    case 2:
    
      if( bState[1]==11 ){
        if( sensorInform[1]<10 ){
          state[1]=0;
        }
      }else if( bState[1]==0 ){ // 針對可能在0度附近震盪  ex 358~2
        if( sensorInform[1]>30 && sensorInform[1]<60 ){
          state[1]=bState[1]+1;
        }
      }else{
        if( sensorInform[1]>limitT[bState[1]+1] ){
            state[1]=bState[1]+1;
        }
      }
      break;
      
    case 3:
      if( bState[1]==0 ){
        if( sensorInform[1]<330 && sensorInform[1]>300 ){
          state[1]=11;
        }
      }else if( bState[1]==1 ){
        if( sensorInform[1]>350 ){
          state[1]=0;
        }
      }else{
        if( sensorInform[1]<limitT[bState[1]-1] ){
          state[1]=bState[1]-1;
        }
      }
      break;      
  }
}

void thetaWall(unsigned short *v,float *sensorInform){
  int ctrV[2]={0};
  
  minSensor(v,ctrV);
  sensorInform[0]=heron_formula(v[ctrV[0]],v[ctrV[1]]);
  if( ctrV[0]*30+acos(sensorInform[0]/v[ctrV[0]])/3.14*180>360 ){
    sensorInform[1]=ctrV[0]*30+acos(sensorInform[0]/v[ctrV[0]])/3.14*180-360;
  }else{
    sensorInform[1]=ctrV[0]*30+acos(sensorInform[0]/v[ctrV[0]])/3.14*180;
  }
  sensorInform[0]=sensorInform[0]/1000;
}

void minSensor(unsigned short *v,int *minV){
  int i;
  int tempV;
  tempV=v[0];
  minV[0]=0;
  for(i=1;i<12;i++){
    if( tempV>v[i] ){
      tempV=v[i];
      minV[0]=i;
    }
  }
  
  if( v[(minV[0]+12-1)%12]<v[(minV[0]+1)%12] ){
    minV[1]=(minV[0]+12-1)%12;
    i=minV[0];
    minV[0]=minV[1];
    minV[1]=i;
  }else{
    minV[1]=(minV[0]+1)%12;
  }
}

float heron_formula(int a,int b){
  float c,p,s,h;
  
  c=pow((pow(a,2)+pow(b,2)-a*b*pow(3,0.5)),0.5);
  p=(a+b+c)/2;
  s=pow((p*(p-a)*(p-b)*(p-c)),0.5);
  h=2*s/c;
   
  return h;
}
