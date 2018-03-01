// 改寫 mybot的範例程式 和 足球範例程式 //

// 功能
// 判斷當前回合數是否滿足訓練次數(最大回合數)//
// 將當前的機器人的位置告知個別機器人 //
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <device/robot.h>
#include <device/supervisor.h>
#include <device/emitter.h>

#define ROBOTS 1
#define TIME_STEP 64
#define episode 200

static void reset(void);
static int run(int);
static void set_scores(int , int);

static NodeRef robotN;
static DeviceTag emitter;
static float position[ROBOTS * 2 + 2];

static int ctrEpiNow=0;
static int ctrItr=0;

static void reset(void)
{
    emitter = robot_get_device("emitter");    
    
    robotN = supervisor_node_get_from_def("B1");
    supervisor_field_get(robotN,
                         SUPERVISOR_FIELD_TRANSLATION_X |
                         SUPERVISOR_FIELD_TRANSLATION_Z,
                         &position[0], TIME_STEP);    
    
    FILE *fpItr=fopen("Itr.txt","r");
    if(fpItr != NULL){
      fscanf(fpItr,"%d",&ctrItr);
      fclose(fpItr);
    }    
    
    
    FILE *fpEpiNow=fopen("EpiNow.txt","r");
    if(fpEpiNow != NULL){
      fscanf(fpEpiNow,"%d",&ctrEpiNow);
      fclose(fpEpiNow);
    }
    
    set_scores(ctrItr,ctrEpiNow);
    
    return;
}

static void set_scores(int b,int b2)
{
    char score[30];    
    sprintf(score,"Expert");
    supervisor_set_label(0, score, 0.3, 0.01, 0.07, 0x0000ff);
    return;
}

static int run(int ms)
{
  // ***** 計時 ***** //
  static float time = 2 * 60;
  static float ball_reset_timer = 0;
  // ***** 旗標 ***** //  
  static int flagGoal=0;
  static int flagTimeUp=0;
  static int flagEpi=0;
  //robot_console_printf("time=%f",time);
  float *buffer;
  buffer = (float *) emitter_get_buffer(emitter);
  
  if(ctrEpiNow == episode){
    flagEpi=1;
  }  
  
  buffer[0] = position[0];    /* robot i: X */
  buffer[1] = position[1];    /* robot i: Z */
  buffer[2] = (float)flagTimeUp;
  buffer[3] = (float)flagGoal;
  buffer[4] = (float)flagEpi;
  
  
  emitter_send(emitter, 5 * sizeof(float));
  time -= (float) TIME_STEP / 1000;
  if (ball_reset_timer == 0) {
    // ***** 判斷是否要reset ***** //
    // ******* 球動了 ******* //
    if ( position[0]>-0.59 && position[0]<-0.53 && position[1]<-0.08 && position[1]>-0.13 ) {            
      flagGoal = 1;
      ball_reset_timer=1;
    }
    // ******* 時間到了 ******* //
    if(time < 0){
      robot_console_printf("forced end\n");
      flagTimeUp=1;
      ball_reset_timer=1;
    }
    
  }else{
    ball_reset_timer -= (float) TIME_STEP / 1000;
    if (ball_reset_timer < 0){
        
      ctrEpiNow++;
      FILE *fpEpiNow = fopen("EpiNow.txt","w");
      fprintf(fpEpiNow,"%d\n",ctrEpiNow);
      fclose(fpEpiNow);
      
      if(flagEpi==1){
        robot_console_printf("DELing");
        //system("PAUSE");
        system("del EpiNow.txt");
        system("del ..\\soccer_player\\Q_table.txt");
        system("del ..\\soccer_player\\step.txt");
        flagEpi=0;
        ctrItr++;
        FILE *fpItr = fopen("Itr.txt","w");
        fprintf(fpItr,"%d\n",ctrItr);
        fclose(fpItr);
        robot_console_printf("DEL success.\n");
        
      }
      set_scores(ctrItr,ctrEpiNow);
      ball_reset_timer=10;
      supervisor_simulation_physics_reset();
      supervisor_simulation_revert();
            
    }
  }
    return TIME_STEP;
}

int main()
{
    robot_live(reset);
    robot_run(run);             /* never returns */

    return 0;
}
