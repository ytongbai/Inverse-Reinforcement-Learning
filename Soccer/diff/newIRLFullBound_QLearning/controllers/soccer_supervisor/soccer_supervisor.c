// 參考足球機器人的範例程式 //

// 功能
// 判斷訓練結束和球是否有進 //
// 將機器人的位置傳給機器人 //
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <device/robot.h>
#include <device/supervisor.h>
#include <device/emitter.h>

#define ROBOTS 1
#define TIME_STEP 64
#define episode 1000

static void reset(void);
static int run(int);


static NodeRef robotN;
static NodeRef ball;
static DeviceTag emitter;
static float position[4];

static int ctrIter=0;
static int ctrEpiNow=0;

static void reset(void)
{
    emitter = robot_get_device("emitter");    
    
    robotN = supervisor_node_get_from_def("B1");
    supervisor_field_get(robotN,
                         SUPERVISOR_FIELD_TRANSLATION_X |
                         SUPERVISOR_FIELD_TRANSLATION_Z,
                         &position[0], TIME_STEP);
    

    ball = supervisor_node_get_from_def("BALL");
    supervisor_field_get(ball,
                         SUPERVISOR_FIELD_TRANSLATION_X |
                         SUPERVISOR_FIELD_TRANSLATION_Z,
                         &position[2], TIME_STEP);
    
        
    
    
    FILE *fpEpiNow=fopen("EpiNow.txt","r");
    if(fpEpiNow != NULL){
      fscanf(fpEpiNow,"%d",&ctrEpiNow);
      fclose(fpEpiNow);
    }
    
    FILE *fpIter=fopen("Iter.txt","r");
    if(fpIter != NULL){
      fscanf(fpIter,"%d",&ctrIter);
      fclose(fpIter);
    }
    
    supervisor_set_label(5,"Mid", 0.3, 0.01, 0.07, 0x0000ff);
    
    return;
}



static int run(int ms)
{
  // ***** 計時 ***** //
  static float time = 5 * 60;
  static float ball_reset_timer = 0;
  // ***** 旗標 ***** //  
  static int flagGoal=0;
  static int flagKick=0;
  static int flagEpi=0;
  
  // 傳送值 //
  float *buffer;
  buffer = (float *) emitter_get_buffer(emitter);
  
  // 判斷訓練是否結束 //
  if( flagEpi==0 ){
    if( ctrEpiNow==episode ){
      flagEpi=1;
      system("PAUSE");
    }
  }
  
  buffer[0] = position[0];    /* robot i: X */
  buffer[1] = position[1];    /* robot i: Z */    
  buffer[2] = position[2];  /* ball X */
  buffer[3] = position[3];  /* ball Z */
  buffer[4] = (float)flagKick;
  buffer[5] = (float)flagGoal;
  buffer[6] = (float)flagEpi; 
  
  emitter_send(emitter, 7 * sizeof(float));
  
  // 讀取專家的動作 //
  FILE *fpQAction = fopen("..//soccer_player//nowQ.txt","r");
  int numQA,setA;
  float nowQ[4]={0};
  if( fpQAction!=NULL ){
    for(numQA=0;numQA<4;numQA++){
      fscanf(fpQAction,"%f",&nowQ[numQA]);
    }
  }
  fscanf(fpQAction,"%d",&setA);
  fclose(fpQAction);
  // 顯示 //
  char QA[10];
  for(numQA=0;numQA<4;numQA++){
    if(numQA!=setA){
      switch(numQA){
        case 0:
          sprintf(QA,"F:%f",nowQ[numQA]);
          supervisor_set_label(0, QA, 0.12, 0.11, 0.05, 0x0000ff);
          break;
        case 1:
          sprintf(QA,"B:%f",nowQ[numQA]);
          supervisor_set_label(1, QA, 0.12, 0.21, 0.05, 0x0000ff);
          break;
        case 2:
          sprintf(QA,"L:%f",nowQ[numQA]);
          supervisor_set_label(2, QA, 0.12, 0.31, 0.05, 0x0000ff);
          break;
        case 3:
          sprintf(QA,"R:%f",nowQ[numQA]);
          supervisor_set_label(3, QA, 0.12, 0.41, 0.05, 0x0000ff);
          break;
      }
    }else{
      switch(numQA){
        case 0:
          sprintf(QA,"F:%f",nowQ[numQA]);
          supervisor_set_label(0, QA, 0.02, 0.11, 0.05, 0xff0000);
          break;
        case 1:
          sprintf(QA,"B:%f",nowQ[numQA]);
          supervisor_set_label(1, QA, 0.02, 0.21, 0.05, 0xff0000);
          break;
        case 2:
          sprintf(QA,"L:%f",nowQ[numQA]);
          supervisor_set_label(2, QA, 0.02, 0.31, 0.05, 0xff0000);
          break;
        case 3:
          sprintf(QA,"R:%f",nowQ[numQA]);
          supervisor_set_label(3, QA, 0.02, 0.41, 0.05, 0xff0000);
          break;
      }    
    }
  }
  
  //supervisor_set_label(0, QA, 0.22, 0.11, 0.05, 0x0000ff);
  
  time -= (float) TIME_STEP / 1000;
  if (ball_reset_timer == 0) {
    // ***** 判斷是否要reset ***** //
    // ******* 球動了 ******* //
    if ((fabs(position[2])-0.56) > 0.0001 || fabs(position[3]) > 0.0001) {  /* ball in the blue goal */            
      ball_reset_timer = 20;   /* wait for 3 seconds before reseting the ball */ 
      flagKick = 1;
      
    }
    // ******* 時間到了 ******* //
    if(time < 0){
      robot_console_printf("forced end\n");
      ball_reset_timer = 1;
      flagKick=1;
    }
    
  }else{
    ball_reset_timer -= (float) TIME_STEP / 1000;
      if(position[2] < -0.74){
        flagGoal = 1;
      }
      
      if (ball_reset_timer < 0){
        system("PAUSE");
        ctrEpiNow++;
        FILE *fpEpiNow = fopen("EpiNow.txt","w");
        fprintf(fpEpiNow,"%d\n",ctrEpiNow);
        fclose(fpEpiNow);
        //system("PAUSE");
        ball_reset_timer=20;
		robot_console_printf("a");
        supervisor_simulation_physics_reset();
        supervisor_simulation_revert();            
      }
	  robot_console_printf("c");
  }
    return TIME_STEP;
}

int main()
{
    robot_live(reset);
    robot_run(run);             /* never returns */

    return 0;
}
