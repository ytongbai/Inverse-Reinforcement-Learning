// 參考足球機器人的範例程式 //

// 功能
// 判斷訓練結束(時間到或是球移動)和球是否有進 //
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
#define episode 500

static void reset(void);
static int run(int);
// 輸出於world windows (包含了當前疊代次數和回合數)
static void set_scores(int , int);

static NodeRef robotN;
static NodeRef ball;
static DeviceTag emitter;
static float position[4];

// ctr為counter
// ctrIter為目前疊代次數
// ctrIter為目前回合次數
static int ctrIter=0;
static int ctrEpiNow=0;

static void reset(void)
{
    robot_console_printf("SupV resting");
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
    
    // 讀取上一回合的疊代次數和回合數
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
    
    set_scores(ctrIter , ctrEpiNow);
    robot_console_printf("SupV rest done\n");
    return;
}

static void set_scores(int b,int b2)
{
    char score[40];
    sprintf(score,"Iteration:%03d Episode:%03d",b,b2);
    supervisor_set_label(0, score, 0.22, 0.01, 0.07, 0x0000ff);
    return;
}

static int run(int ms)
{
  // ***** 計時 ***** //
  static float time = 5 * 60;
  static float ball_reset_timer = 0;
  // ***** 旗標 ***** // 
  // flagGoal球是否進球門
  // flagKick球是否移動
  // flagEpi訓練是否結束(以達到最大回和數)  
  static int flagGoal=0;
  static int flagKick=0;
  static int flagEpi=0;
  
  // 傳送當前機器人的位置、球的位置和各個旗標
  float *buffer;
  buffer = (float *) emitter_get_buffer(emitter);
  
  // 訓練完成開啟旗標
  if(ctrEpiNow == episode){
    flagEpi=1;
  }  
  
  buffer[0] = position[0];    /* robot i: X */
  buffer[1] = position[1];    /* robot i: Z */    
  buffer[2] = position[2];  /* ball X */
  buffer[3] = position[3];  /* ball Z */
  buffer[4] = (float)flagKick;
  buffer[5] = (float)flagGoal;
  buffer[6] = (float)flagEpi; 
  
  emitter_send(emitter, 7 * sizeof(float));
  
  // 做計時 
  time -= (float) TIME_STEP / 1000;
  // 回合是否結束
  if (ball_reset_timer == 0) {
    // ******* 球動了 ******* //
    if ((fabs(position[2])-0.56) > 0.0001 || fabs(position[3]) > 0.0001) {
      // 等待20秒確保球有機會滾進球門
      ball_reset_timer = 20;   
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
    // 判斷球是否進球門
    if(position[2] < -0.74){
      flagGoal = 1;
    }

    if (ball_reset_timer < 0){
      // 回合數+1
      ctrEpiNow++;
      FILE *fpEpiNow = fopen("EpiNow.txt","w");
      fprintf(fpEpiNow,"%d\n",ctrEpiNow);
      fclose(fpEpiNow);
      
      // 將Qtable和回合數歸零 並 將疊代次數+1
      if(flagEpi==1){
      //system("PAUSE");
        ctrIter++;
        FILE *fpIter = fopen("Iter.txt","w");
        fprintf(fpIter,"%d\n",ctrIter);
        fclose(fpIter);
        robot_console_printf("DELing");
        system("del EpiNow.txt");
        system("del ..\\soccer_player\\Q_table.txt");
        system("del ..\\soccer_player\\step.txt");
        flagEpi=0;
        robot_console_printf("DEL success.\n");
      }
      ball_reset_timer=20;
      robot_console_printf("a");
      // 將機器人和球回到原本的位置
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
