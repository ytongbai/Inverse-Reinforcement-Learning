// ��g mybot���d�ҵ{�� �M ���y�d�ҵ{�� //

// �\��
// �P�_��e�^�X�ƬO�_�����V�m����(�̤j�^�X��)//
// �N��e�������H����m�i���ӧO�����H //
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
#define episodeSecond 5

static void reset(void);
static int run(int);
// ��X��world windows (�]�t�F��e�|�N���ƩM�^�X��)
static void set_scores(int , int);

static NodeRef robotN;
static DeviceTag emitter;
static float position[ROBOTS * 2 + 2];

// ctr��counter
// ctrIter���ثe�|�N����
// ctrIter���ثe�^�X����
static int ctrEpiNow=0;
static int ctrItr=0;

static void reset(void)
{ 
    robot_console_printf("SupV reseting..");
    emitter = robot_get_device("emitter");    
    
    robotN = supervisor_node_get_from_def("B1");
    supervisor_field_get(robotN,
                         SUPERVISOR_FIELD_TRANSLATION_X |
                         SUPERVISOR_FIELD_TRANSLATION_Z,
                         &position[0], TIME_STEP);    
    
    // Ū���W�@�^�X���|�N���ƩM�^�X��
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
    robot_console_printf("SupV reset done\n");
    return;
}

// ��X��e�^�X�� �M ���a����
static void set_scores(int b,int b2)
{
    char score[30];    
    sprintf(score,"iteration:%d  episode:%d",b,b2);
    supervisor_set_label(0, score, 0.3, 0.01, 0.07, 0x0000ff);
    return;
}

static int run(int ms)
{
  // ***** �p�� ***** //
  static float time = episodeSecond * 60;
  static float ball_reset_timer = 0;
  // ***** �X�� ***** //  
  // flagGoal�y�O�_�i�y��
  // flagKick�y�O�_����
  // flagEpi�V�m�O�_����(�H�F��̤j�^�M��)  
  static int flagGoal=0;
  static int flagTimeUp=0;
  static int flagEpi=0;
  // ***** catch buffer ***** //
  float *buffer;
  buffer = (float *) emitter_get_buffer(emitter);
  // ***** �P�_ �V�m���ƬO�_�H�F�� ****** //
  if(ctrEpiNow == episode){
    flagEpi=1;
  }  
  
  // �e�X�T�� //
  buffer[0] = position[0];    /* robot i: X */
  buffer[1] = position[1];    /* robot i: Z */
  buffer[2] = (float)flagTimeUp;
  buffer[3] = (float)flagGoal;
  buffer[4] = (float)flagEpi;  
  emitter_send(emitter, 5 * sizeof(float)); 
  
  // ���p�� 
  time -= (float) TIME_STEP / 1000;
  // �^�X�O�_����
  if (ball_reset_timer == 0) {
    // �P�_�O�_�w����ؼ� //
    if ( position[0]>-0.59 && position[0]<-0.53 && position[1]<-0.08 && position[1]>-0.13 ) {            
      flagGoal = 1;
      ball_reset_timer=1;
    } 
    // �ɶ���F //
    if(time < 0){
      robot_console_printf("forced end\n");
      flagTimeUp=1;
      ball_reset_timer=1;
    }
    
  }else{
    ball_reset_timer -= (float) TIME_STEP / 1000;
    if (ball_reset_timer < 0){
      // �^�X��+1
      ctrEpiNow++;
      FILE *fpEpiNow = fopen("EpiNow.txt","w");
      fprintf(fpEpiNow,"%d\n",ctrEpiNow);
      fclose(fpEpiNow);
      
      // �NQtable�M�^�X���k�s �� �N�|�N����+1
      if(flagEpi==1){
        //system("PAUSE");
        robot_console_printf("DELing");
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
      ball_reset_timer=1;
      robot_console_printf("a");
      // �N�����H�M�y�^��쥻����m
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
