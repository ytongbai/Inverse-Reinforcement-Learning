// 參考足球機器人的範例程式 //

// 功能
// Qlearning(policy,mu1)->update funciton(ada weight)->IRL(reward) //

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <device/robot.h>
#include <device/differential_wheels.h>
#include <device/receiver.h>
#include <device/gps.h>
// 參數設定
// arc為學習率 gama為折扣率 exp_rate為探索率
#define arc 0.7
#define gama 0.7
#define exp_rate 0.8
// feature_num為總共特徵數量 sNum為總共狀態數量 rNum為距離的狀態切割數量 tNum為角度的狀態切割數量
// actNum為動作的數量
#define feature_num 180
#define sNum 180
#define rNum 10
#define tNum 18
#define actNum 4

// 球的初始位置
#define ballS1 -0.56
#define ballS2 0

#include <Q_func.h>
#include <adaboost_tableP.h>
#define ROBOTS 1
#define TIME_STEP 64

#define desireError 0.0001 // 預期與專家間的差距
#define gamaIrl gama


// ***** function ***** //
static void reset(void);
static int run(int);
void Fstate(float,float,int *,int *); // 位置轉換為狀態
void InitFstate(float,float,int *); // 初始位置轉換為狀態
float CalTheta(float,float,float,float); // 機器人正面與機器人到球之間的夾角
// ***** 變數 ***** //
// ********** 裝置 ********** //
static DeviceTag receiver;      
static DeviceTag gps;
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
  int i,j;
  robot_console_printf("reseting");  

  receiver = robot_get_device("receiver");
  receiver_enable(receiver, TIME_STEP);
  gps = robot_get_device("gps");
  gps_enable(gps,TIME_STEP);   
  
  srand(time(NULL));
  
  
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
  robot_console_printf("reset done\n");  
}

static int run(int ms)
{
  const float *buffer; // supervisor傳送來的資訊
  const float *gps_information; // gps抓取到的資訊(機器人白點的 x和 z)，計算角度用
  
  static int flagStepOne=0; // 讓步數只記錄一次
  static int flagGoalOne=0; // 讓Q_table只記錄一次除非球進
  static int flagMu1One=0; // 紀錄mu_1已經計算結束
  static int flagWallOne=0; // 紀錄此時為撞牆
  
  static int ctrWall=0; // 紀錄狀態不變化的次數 當大於某個值判斷為撞牆
  static int ctrStep=0; // 紀錄該回合使用的步數
  static int ctrInput=0;// 由於剛開始supervisor傳送的值未必正確 因此從第二次傳入的值開始做計算
  static int ctrTrajectory=0; // 計算mu值作為紀錄第幾步用
  static int ctrKeepGo=0;
  
  static int bState[2]={0}; // 前一個狀態
  static int bAction=1; // 上次選擇的動作
  int state[2],action; // 當前狀態與與這次選取的動作
  
  // 左右輪子的速度
  int left_speed=0; 
  int right_speed=0;
  
  float position_x,position_z; //gps抓到的值
  float tempQ;
  float endQ[actNum]={0};
  float errorNow=0; //專家與學習出來的mu之間的距離
  int n,i,j;
  float theta=0;
  left_speed = 0;
  right_speed = 0;
  
  
  n = receiver_get_buffer_size(receiver);
  
  if(n  && flagMu1One==0){
    // ***** 將Input轉為State  並計算action ***** //
    gps_information = (const float *) gps_get_matrix(gps);
    position_x = gps_position_x(gps_information);
    position_z = gps_position_z(gps_information);
    
    buffer = (const float *) receiver_get_buffer(receiver);
    
    // 初始化
    if(ctrInput == 1){
      InitFstate(buffer[0],buffer[1],bState);
      bAction=greed_selete(tableQ[bState[0]*tNum+bState[1]]);      
    }
    if(ctrInput > 1){
      Fstate(buffer[0],buffer[1],state,bState);
      theta=CalTheta(buffer[0],buffer[1],position_x,position_z);
      if(buffer[6]<0.5){ // 判斷訓練是否結束
        action=e_greed_selete(tableQ[state[0]*tNum+state[1]]);
      
        if(buffer[4]<0.5){ // 判斷ball有沒有動
          if(state[0]==bState[0] && state[1]==bState[1]){ // 判斷狀態有沒有變
            ctrWall=ctrWall+1;
            
            if(ctrWall > 30*(bState[0]+12)){ // 判斷在原地使否待太久 作為是否撞牆的依據
              // 撞牆 更新Q_table
              tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[state[0]*tNum+state[1]],omg[sNum]);              
              tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;

              ctrWall=0;              
              bAction=action;
              ctrStep++;
              
            }else{ // 若薇相同狀態則執行相同動作
              action=bAction;
            }    
            
          }else{
            ctrWall=0;
            // 沒有撞牆 更新Q_table
            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[state[0]*tNum+state[1]],omg[state[0]*tNum+state[1]]);
            tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;

            bAction=action;  
            bState[0]=state[0];
            bState[1]=state[1];
            
            ctrStep++;
          }      
          
        }else{
          if(buffer[5]>0.5 && flagGoalOne!=2){
            // 球進 更新Q_table
            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],endQ,omg[state[0]*tNum+state[1]]);
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
          // 記錄step在step.txt
          if(flagStepOne == 0){        
            robot_console_printf("writing the step.  ");
            FILE *fp_step = fopen("step.txt","a");    
            fprintf(fp_step,"%d\n",ctrStep);  
            fclose(fp_step);
            robot_console_printf("success writing the step %d\n",ctrStep);
            flagStepOne = 1;
          }        
          ctrKeepGo++;
        }
      }else{// 計算mu值
        
        action=greed_selete(tableQ[state[0]*tNum+state[1]]);
        if(buffer[4] < 0.5 && flagWallOne != 1){
          if(state[0]==bState[0] && state[1]==bState[1]){
            ctrWall++;
            if(ctrWall > 30*(bState[0]+12)){            
              bAction=action;
              ctrWall=0;
              
              ctrTrajectory++;
              mu1[bState[0]*tNum+bState[1]]=mu1[bState[0]*tNum+bState[1]]+pow(gamaIrl,ctrTrajectory);
              robot_console_printf("stateMove(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],bState[0]*tNum+bState[1],mu1[bState[0]*tNum+bState[1]]);
              robot_console_printf("tableQ[%d][%d]=%f\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
              ctrTrajectory++;
              mu1[sNum]=mu1[sNum]+pow(gama,ctrTrajectory);            
              robot_console_printf("stateWall(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],sNum,mu1[sNum]);
              robot_console_printf("tableQ[%d][%d]=%f\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
              
              flagWallOne=1;
            }
            action=bAction;
          }else{
            ctrWall=0;           
            
            ctrTrajectory++;
            mu1[bState[0]*tNum+bState[1]]=mu1[bState[0]*tNum+bState[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateMove(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],bState[0]*tNum+bState[1],mu1[bState[0]*tNum+bState[1]]);
            robot_console_printf("tableQ[%d][%d]=%f\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
            
            bAction=action;  
            bState[0]=state[0];
            bState[1]=state[1];
          }
        }else{
          if( flagWallOne!=1 ){
            ctrTrajectory++;
            mu1[state[0]*tNum+state[1]]=mu1[state[0]*tNum+state[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateEnd(%d)[%d %d],mu1[%d]=%f\n",ctrTrajectory,state[0],state[1],state[0]*tNum+state[1],mu1[state[0]*tNum+state[1]]);
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
            FILE *fpMu1=fopen("Mu1.txt","w"); 
            for(i=0;i<sNum+1;i++){
              // update omg的值
              omg[i]=omg[i]+(mue[i]-mu1[i])/errorNow*weightAda[i];
              fprintf(fpOmg,"%f\n",omg[i]);
              fprintf(fpMu1,"%f\n",mu1[i]);
            }
            fclose(fpOmg);
            fclose(fpMu1);
            
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
          ctrKeepGo++;
        }
      }
      
      if(buffer[4]<0.5 && flagFinish!=1){ // 球沒移動或時間沒到或是還沒學完成前 針對不同動作控制兩個輪子的轉速
        switch(action){
          case 0: //向球
            if( (theta<1 && theta>0)||(theta<360 && theta>359) ){
              left_speed = 10;
              right_speed = 10;
            }else{
              if(theta<180){
                left_speed = 10;
                right_speed = -10;
              }else{
                left_speed = -10;
                right_speed = 10;
              }
            }
            break;
          case 1: // 逆球
            if(theta<181 && theta>179){
              left_speed = 10;
              right_speed = 10;
            }else{
              if(theta<180){
                left_speed = -10;
                right_speed = 10;
              }else{
                left_speed = 10;
                right_speed = -10;
              }
            }
            break;
          case 2: // 向球往左
            if(theta<91 && theta>89){
              left_speed = 10;
              right_speed = 10;
            }else{
              if(theta<90 || theta>270){
                left_speed = -10;
                right_speed = 10;
              }else{
                left_speed = 10;
                right_speed = -10;
              }
            }
            break;
          case 3: // 向球往右
            if(theta<271 && theta>269){
              left_speed = 10;
              right_speed = 10;
            }else{
              if(theta<270 && theta>90){
                left_speed = -10;
                right_speed = 10;
              }else{
                left_speed = 10;
                right_speed = -10;
              }
            }
            break;
        }

      }else{
        left_speed = 0;
        right_speed = 0;
      }
    }
    ctrInput++;
  }
  
  if(ctrKeepGo==1 && !flagMu1One){
    left_speed=10;
    right_speed=10;   
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

void Fstate(float s1,float s2,int *state,int *bState){
	int i;

  float temp=0;
  // ***** radious num 10 ****** //
  float limit[10]={0.0585,0.085,0.12,0.15,0.21,0.28,0.36,0.45,0.55,0.75};
  float boundR=0.002;
  float boundT=0.1;
  int bound[3]={0};
  temp=pow((float)(pow(s1-ballS1,2)+pow(s2-ballS2,2)),0.5);
  
  for(i=0;i<10;i++){
    if(temp<limit[i]){
      state[0]=i;
      break;
    }else{
      state[0]=9;
    }
  }

  if(i>0){
    if( temp<(limit[i-1]+boundR) || temp>(limit[i]-boundR) ){
      state[0]=bState[0];
    }
  }
  
  // ****** theta num 18 ***** //
  if(s1==ballS1 && s2==ballS2){
    state[1]=0;
  }else{
    if((s2-ballS2)>0){
      bound[0]=acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14/20;
      bound[1]=(acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14+boundT)/20;
      bound[2]=(acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14-boundT)/20;
      if(bound[0]==bound[1] && bound[0]==bound[2]){
        state[1]=bound[0];
      }else{
        state[1]=bState[1];
      }
    }else{    
      bound[0]=(360-acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14)/20;
      bound[1]=(360-acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14+boundT)/20;
      bound[2]=(360-acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14-boundT)/20;
      if(bound[0]==bound[1] && bound[0]==bound[2]){
        state[1]=bound[0];
      }else{
        state[1]=bState[1];
      }
    }
  }
  
}

void InitFstate(float s1,float s2,int *state){
	int i;

  float temp=0;
  // ***** radious num 13 ****** //
  float limit[10]={0.0585,0.085,0.12,0.15,0.21,0.28,0.36,0.45,0.55,0.75};

  temp=pow((float)(pow(s1-ballS1,2)+pow(s2-ballS2,2)),0.5);
  
  for(i=0;i<10;i++){
    if(temp<limit[i]){
      state[0]=i;
      break;
    }
  }

  // ****** theta num 18 ***** //
  if(s1==ballS1 && s2==ballS2){
    state[1]=0;
  }else{
    if((s2-ballS2)>0){
      state[1]=acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14/20;
    }else{
      state[1]=(360-acos((1)*(s1-ballS1)/pow((pow((s1-ballS1),2)+pow((s2-ballS2),2)),0.5))*180/3.14)/20;
    }
  }
}

float CalTheta(float x,float y,float xGps,float yGps){

  const float xBall=-0.56;
  const float yBall=0;
  float theta=0;
  float tempUpper,tempLower;
  
  tempUpper=((xBall-x)*(xGps-x)+(yBall-y)*(yGps-y));
  tempLower=pow((pow((xBall-x),2)+pow((yBall-y),2)),0.5)*pow((pow((xGps-x),2)+pow((yGps-y),2)),0.5);
  theta=acos(tempUpper/tempLower)*180/3.14;
  
  if( (yBall-y)*xGps-(xBall-x)*yGps < (yBall-y)*xBall-(xBall-x)*yBall ){   
    theta=360-theta;
  }
  return theta;
}
