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
#define episode 500

static void reset(void);
static int run(int);
static void set_scores(void);

static NodeRef robotN;
static NodeRef ball;
static DeviceTag emitter;
static float position[4];
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
    
    set_scores();
    
    return;
}

static void set_scores(void)
{
  supervisor_set_label(0, "Expert" , 0.52, 0.01, 0.07, 0x0000ff);
  return;
}

static int run(int ms)
{
  // ***** 計時 ***** //
  static float time = 10 * 60;
  static float ball_reset_timer = 0;
  // ***** 旗標 ***** //  
  static int flagGoal=0;
  static int flagKick=0;
  static int flagEpi=0;
  //robot_console_printf("time=%f",time);
  float *buffer;
  buffer = (float *) emitter_get_buffer(emitter);
  
  if(ctrEpiNow==episode){
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
  time -= (float) TIME_STEP / 1000;  
  if (ball_reset_timer == 0) {
    // ***** 判斷是否要reset ***** //
      // ******* 球動了 ******* //
    if ( (fabs(position[2])-0.56)>0.0001 || fabs(position[3])>0.0001 ) {
        // ***** 等20s，確定球進 ****** // 
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
    ball_reset_timer-=(float) TIME_STEP / 1000;
            
    if ( ball_reset_timer<0 ){
      
      // ***** 讓球和機器人有時回到原本的點 ***** //
      ball_reset_timer=20;
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
