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

#define arc 0.7
#define gama 0.7
#define episode 100
#define trail 10
#define exp_rate 0.7

#define feature_num 180
#define sNum 180
#define rNum 10
#define tNum 18
#define aNum 8
#define actNum 4

#define ballS1 -0.56
#define ballS2 0

#include <Q_func.h>
#include <adaboost_tableP.h>
#define ROBOTS 1
#define TIME_STEP 64

#define desireError 0.0001
#define gamaIrl gama
#define numFeature sNum+1

// ***** function ***** //
static void reset(void);
static int run(int);
void Fstate(float,float,int *,int *);
void InitFstate(float,float,int *);
float CalTheta(float,float,float,float);
// ***** 變數 ***** //
// ********** 裝置 ********** //
static DeviceTag receiver;      
static DeviceTag gps;
// ********** 值 ********** //
static float tableQ[sNum][actNum]={{0}};
static float mu1[sNum+1]={0};
static float omg[sNum+1]={0};
//static int keyInput[192]={0}; 

// ********** 旗標 ********** //
static int flagFinish=0;

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
  
  robot_console_printf("reset done\n");  
}

static int run(int ms)
{
  const float *buffer;
  const float *gps_information; 
  
  static int flagStepOne=0;
  static int flagGoalOne=0;
  static int flagMu1One=0;
  static int flagWallOne=0;
  
  static int ctrWall=0;
  static int ctrStep=0;
  static int ctrInput=0;
  static int ctrTrajectory=0;
  static int ctrKeepGo=0;
  
  static int bState[2]={0};
  static int bAction=1;
  //static int ctrAction=0;
  int state[2],action;
  
  int left_speed=0;
  int right_speed=0;
  
  float position_x,position_z;
  float tempQ;
  float endQ[actNum]={0};
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
    
    if(ctrInput == 1){
      InitFstate(buffer[0],buffer[1],bState);
      bAction=greed_selete(tableQ[bState[0]*tNum+bState[1]]);
      // ******************** //
      FILE *Road = fopen("road.txt","w");
      fprintf(Road,"%d %d\n",bState[0],bState[1]);
      fclose(Road);
      // ******************** //
    }
    if(ctrInput > 1){
      Fstate(buffer[0],buffer[1],state,bState);
      theta=CalTheta(buffer[0],buffer[1],position_x,position_z);
      if(buffer[6]<0.5){ // 判斷episode
        action=e_greed_selete(tableQ[state[0]*tNum+state[1]]);
        
        if(buffer[4]<0.5){ // 判斷ball有沒有動
          if(state[0]==bState[0] && state[1]==bState[1]){
            ctrWall=ctrWall+1;
            
            if(ctrWall > 30*(bState[0]+12)){

              tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[state[0]*tNum+state[1]],omg[sNum]);              
              tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;

              ctrWall=0;              
              bAction=action;
              ctrStep++;
              
            }else{
              action=bAction;
            }    
            
          }else{
            ctrWall=0;

            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],tableQ[state[0]*tNum+state[1]],omg[state[0]*tNum+state[1]]);
            tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;

            bAction=action;  
            bState[0]=state[0];
            bState[1]=state[1];
            
            ctrStep++;
          }      
          
        }else{

          if(buffer[5]>0.5 && flagGoalOne!=2){

            tempQ=updateQ_table(tableQ[bState[0]*tNum+bState[1]][bAction],endQ,omg[state[0]*tNum+state[1]]);
            tableQ[bState[0]*tNum+bState[1]][bAction]=tempQ;
            
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
      }else{
        //system("PAUSE");
        action=greed_selete(tableQ[state[0]*tNum+state[1]]);
       
       // ***************************** //       
        
        FILE *fpQAction = fopen("nowQ.txt","w");
        int numQA;
        for(numQA=0;numQA<actNum;numQA++){
          fprintf(fpQAction,"%f\n",tableQ[state[0]*tNum+state[1]][numQA]);
        }
        fprintf(fpQAction,"%d\n",action);
        fclose(fpQAction);
        // ******************************* //
        if(buffer[4] < 0.5 && flagWallOne != 1){
          if(state[0]==bState[0] && state[1]==bState[1]){
            ctrWall++;
            if(ctrWall > 30*(bState[0]+12)){            
              bAction=action;
              ctrWall=0;
              FILE *fpState = fopen("state.txt","a");
              fprintf(fpState,"%02d %02d\n",bState[0],bState[1]);
              fclose(fpState);
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
            FILE *fpState = fopen("state.txt","a");
            fprintf(fpState,"%02d %02d\n",bState[0],bState[1]);
            fclose(fpState);
            ctrTrajectory++;
            mu1[bState[0]*tNum+bState[1]]=mu1[bState[0]*tNum+bState[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateMove(%d)[%d %d],mu1[%d]=%f",ctrTrajectory,bState[0],bState[1],bState[0]*tNum+bState[1],mu1[bState[0]*tNum+bState[1]]);
            robot_console_printf("tableQ[%d][%d]=%f\n",bState[0]*tNum+bState[1],bAction,tableQ[bState[0]*tNum+bState[1]][bAction]);
            
            bAction=action;  
            bState[0]=state[0];
            bState[1]=state[1];
            // ********************************* //
            FILE *Road = fopen("road.txt","a");
            fprintf(Road,"%d %d\n",bState[0],bState[1]);
            fclose(Road);
            // ********************************* //
          }
        }else{
          if( flagWallOne!=1 ){
            FILE *fpState = fopen("state.txt","a");
            fprintf(fpState,"%02d %02d\n",bState[0],bState[1]);
            fclose(fpState);
            ctrTrajectory++;
            mu1[state[0]*tNum+state[1]]=mu1[state[0]*tNum+state[1]]+pow(gamaIrl,ctrTrajectory);
            robot_console_printf("stateEnd(%d)[%d %d],mu1[%d]=%f\n",ctrTrajectory,state[0],state[1],state[0]*tNum+state[1],mu1[state[0]*tNum+state[1]]);
            // ********************************* //
            FILE *Road = fopen("road.txt","a");
            fprintf(Road,"%d %d\n",state[0],state[1]);
            fclose(Road);
            // ********************************* //
          }          
          
          flagMu1One=1;
          ctrKeepGo++;
        }
      }

      if(buffer[4]<0.5 && flagFinish!=1){
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
  // ***** radious num 13 ****** //
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
  float limit[10]={0.065,0.085,0.12,0.15,0.21,0.28,0.36,0.45,0.55,0.75};

  temp=pow((float)(pow(s1-ballS1,2)+pow(s2-ballS2,2)),0.5);
  
  for(i=0;i<10;i++){
    if(temp<limit[i]){
      state[0]=i;
      break;
    }else{
      state[0]=9;
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

