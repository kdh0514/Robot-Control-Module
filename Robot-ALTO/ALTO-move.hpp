
#include <cstdio>
#include <sstream>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "silver_msgs/IoTCallRobot.h"
#include "silver_msgs/IoTEmergStatus.h"

#include "silver_msgs/IoTEnAct.h"
#include "silver_msgs/IoTEnApp.h"
#include "silver_msgs/IoTEnWearable.h"
#include "silver_msgs/IoTGoodMorning.h"
#include "silver_msgs/IoTGoodNight.h"
#include "silver_msgs/IoTHealthReport.h"
#include "silver_msgs/IoTIndoorTempCare.h"
#include "silver_msgs/IoTJustMove.h"
#include "silver_msgs/IoTLifePatternReport.h"
#include "silver_msgs/IoTMealRecommend.h"
#include "silver_msgs/IoTStressCare.h"
#include "silver_msgs/IoTGreeting.h"
#include "silver_msgs/IoTM2MDevice.h"

#include "silver_msgs/RobotState.h"                 
#include "silver_msgs/IoTWarningStatus.h"           
#include "silver_msgs/ComeBackBTN.h"                
#include "silver_msgs/IoTAltoMove.h"               

#include "astra_skeleton/msgGaitEvent.h"
#include "astra_skeleton/msgGaitInform.h"
#include "astra_skeleton/msgGaitParameter.h"
#include "astra_skeleton/msgJointPosition.h"
#include "astra_skeleton/msgSkeleton.h"
#include "silver_msgs/msgSkeletonStatus.h"

#include "silver_msgs/RobotMode.h"
#include "silver_msgs/RobotTouch.h"
#include "silver_msgs/RobotPerson.h"
#include "silver_msgs/RobotPosition.h"
#include "silver_msgs/TabletStatus.h"
#include "silver_msgs/VoiceResult.h"
#include "silver_msgs/IoTDailyRoutine.h"

 
#include "gopher_delivery_msgs/DeliveryGoal.h"
#include "gopher_delivery_msgs/DeliveryStatus.h"

#if 1 // _shjang_
#include "gopher_cns_msgs/Instr.h"
#include "gopher_cns_msgs/Job.h"
#include "gopher_cns_msgs/JobCommand.h"
#include "gopher_cns_msgs/AssignInstrActionResult.h"
#endif

#include "isr_people_tracker/Status.h"
#include "isr_people_tracker/Persons.h"
#include "isr_people_tracker/Person.h"

#include "isr_people_tracker/FollowingCmd.h"

#include "silver_app/silver_app.h"
#define STR_SUCCESS "success"
#define STR_FAIL "fail"

class SilverApplication {
public:
    SilverApplication():
    loop_count(0),
    sequence(SEQ_PUB_INIT),
    nIot_evt(IOT_EVT_NONE),
    nEmerg_evt(0),
    bRobot_Control(true),
    bTbl_emerge_userOk(false),
    bTbl_emerg_modeOff(false),
    bTbl_family_userOut(false),     // default, user in.
    bTbl_family_userOk(false),
    bTbl_family_modeOff(false),
    bTbl_warning_userWearabled(false),
    bTbl_warning_modeOff(false),
    bTbl_warning_userSleep(false),
    bTbl_repoting_modeOff(false),
    bTbl_esc(false),
    nDelivFeed_state(0),
    skeleton_auto_finish(false),
    greeting_mode(false),
    greeting_following_mode(false),
    standby_mode(false),
    dailyroutine_sequence(SEQ_DAILYROUTINE_MAX),
    nDeliv_result(1)        // default, gopher_delivery_msgs '0' means success.
    {
        const char * greeting = getenv("SILVER_GREETING_MODE");
        if (strcmp("true", greeting) == 0)
        {
            ROS_INFO("GREETING MODE is TRUE");
            //greeting_mode = true;
            greeting_following_mode = true;
           
        }
        else
        {
            ROS_INFO("GREETING MODE is FALSE");
            //greeting_mode = false;
            greeting_following_mode = false;
        }

        pub_robotStatus     = nh.advertise<silver_msgs::RobotMode>("robot_mode", 1000);
        pub_robotTouch      = nh.advertise<silver_msgs::RobotTouch>("robot_touch", 1000);
        pub_robotPosition   = nh.advertise<silver_msgs::RobotPosition>("robot_position", 1000);
        pub_newPerson       = nh.advertise<silver_msgs::RobotPerson>("new_person", 1000);
        pub_gpTeleop        = nh.advertise<geometry_msgs::Twist>("/core/yocs_cmd_vel_mux/input/teleop", 1000);

        pub_reporting       = nh.advertise<silver_msgs::IoTWarningStatus>("reporting", 1000);                                  
        pub_robotstate      = nh.advertise<silver_msgs::RobotState>("robot_state", 1000);                                  
        pub_comeback_btn    = nh.advertise<silver_msgs::ComeBackBTN>("pubcomebackbtn", 1000);                           
   
        sub_iotStatus_cr  = nh.subscribe("/iot_status/calling_robot", 1000, &SilverApplication::iotCallRobotCallback, this);
        sub_iotStatus_mm  = nh.subscribe("/iot_status/device_value", 1000, &SilverApplication::iotM2MDeviceCallback, this);
        sub_iotStatus_gr  = nh.subscribe("/iot_status/greeting_status", 1000, &SilverApplication::iotGreetingCallback, this);
        sub_iotStatus_em  = nh.subscribe("/iot_status/emergency", 1000, &SilverApplication::iotEmergCallback, this);
        sub_iotStatus_ac  = nh.subscribe("/iot_status/enact", 1000, &SilverApplication::iotEnactCallback, this);
        sub_iotStatus_ap  = nh.subscribe("/iot_status/en_app", 1000, &SilverApplication::iotEnappCallback, this);
        sub_iotStatus_wb  = nh.subscribe("/iot_status/en_wearable", 1000, &SilverApplication::iotEnWearableCallback, this);
        sub_iotStatus_gm  = nh.subscribe("/iot_status/morning_msg", 1000, &SilverApplication::iotGoodMorningCallback, this);
        sub_iotStatus_gn  = nh.subscribe("/iot_status/good_night", 1000, &SilverApplication::iotGoodNightCallback, this);
        sub_iotStatus_hr  = nh.subscribe("/iot_status/entire_report", 1000, &SilverApplication::iotHealthReportCallback, this);
        sub_iotStatus_tc  = nh.subscribe("/iot_status/indoor_temperature", 1000, &SilverApplication::iotTempCareCallback, this);
        sub_iotStatus_mv  = nh.subscribe("/iot_status/just_move", 1000, &SilverApplication::iotJustMoveCallback, this);
        sub_iotStatus_pr  = nh.subscribe("/iot_status/life_pattern_report", 1000, &SilverApplication::iotLifePatternCallback, this);
        sub_iotStatus_mr  = nh.subscribe("/iot_status/meal_recommend", 1000, &SilverApplication::iotMealRecommendCallback, this);
        sub_iotStatus_sc  = nh.subscribe("/iot_status/stress_care", 1000, &SilverApplication::iotStressCareCallback, this);
        sub_iotStatus_dr  = nh.subscribe("/iot_status/daily_routine", 1000, &SilverApplication::iotDailyRoutineCallback, this);
   
        sub_iotStatus_comeback          = nh.subscribe("/iot_status/comeback", 1000, &SilverApplication::iotComeBackCallback, this);                
        sub_iotStatus_altomove          = nh.subscribe("/iot_status/altomove", 1000, &SilverApplication::iotAltoMoveCallback, this);                 
        sub_iotStatus_subcomebackbtn    = nh.subscribe("/iot_status/subcomebackbtn", 1000, &SilverApplication::iotSubComeBackBTNCallback, this);     
        sub_iotStatus_reporting         = nh.subscribe("/iot_status/reporting", 1000, &SilverApplication::iotReportingCallback, this);               

        sub_skeletonStatus  = nh.subscribe("skeleton_status", 1000, &SilverApplication::SkeletonStatusCallback, this);

        sub_tabletStatus  = nh.subscribe("tablet_status", 1000, &SilverApplication::tabletCallback, this);

        // srvClient_goal    = nh.serviceClient<gopher_delivery_msgs::DeliveryGoal>("/behaviours/delivery/goal"); // /behaviours/delivery/goal
        // srvClient_result  = nh.serviceClient<gopher_delivery_msgs::DeliveryStatus>("/behaviours/delivery/status"); //DeliveryFeedback??
        sub_snd_goal  = nh.subscribe("/gopher_cns/instr_assignee/result", 1, &SilverApplication::sndGoalFinishCallback, this);
        srvTask = nh.serviceClient<gopher_cns_msgs::JobCommand>("/gopher_cns/gopher_cerebrum/command_broker/task");

        isr_people_tracker_status = nh.subscribe("/isr_people_tracker/Status", 1000, &SilverApplication::trackerStatusCallback, this);
        isr_people_tracker_person = nh.subscribe("/isr_people_tracker/TargetPerson", 1000, &SilverApplication::trackerPersonCallback, this);
        // isr_people_tracker_persons = nh.subscribe("/isr_people_tracker/Persons", 1000, &SilverApplication::trackerPersonsCallback, this);       
        srv_following_cmd  = nh.serviceClient<isr_people_tracker::FollowingCmd>("/FollowingCmd"); 
    }

    void trackerStatusCallback(const isr_people_tracker::StatusConstPtr &rst_msg)
    {
      // ROS_INFO("in trackerStatusCallback tracking %d|state %s", rst_msg->tracking, rst_msg->state.c_str());
      return;
    }

    void trackerPersonsCallback(const isr_people_tracker::PersonsConstPtr &rst_msg)
    {
      // ROS_INFO("in trackerPersonsCallback id %d", rst_msg->id);
      return;
    }

    void trackerPersonCallback(const isr_people_tracker::PersonConstPtr &rst_msg)
    {
      // ROS_INFO("in trackerPersonCallback id %d", rst_msg->id);
      return;
    }       

    void sndGoalFinishCallback(const gopher_cns_msgs::AssignInstrActionResultConstPtr &rst_msg)
    {
      int wResult = -1;

      ROS_INFO("in sndGoalFinishCallback %d, %d", rst_msg->result.instr.instr_type, rst_msg->result.instr.status);
      if ((rst_msg->result.instr.instr_type == 1 )&&(rst_msg->result.instr.status == 2))
      {
        nDeliv_result = 0;
        sDeliv_errMsg = STR_SUCCESS;
        // wResult = write( sockfd , "N010000\n\r" , strlen( "N010000\n\r" ) );
        // ROS_INFO("write result : %d", wResult);
      }
      else
      {
        nDeliv_result = 1;
        sDeliv_errMsg = STR_FAIL;
      }
    }   

    void iotGreetingCallback(const silver_msgs::IoTGreetingConstPtr &iot_msg)
    {
        ROS_INFO("IoT_greeting_cbk - evt[%d]   pos_id[%d] msg[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        np.check_new_person = 1;
        np.msg_idx=iot_msg->msg_idx;
        pub_newPerson.publish(np);
    }

    void makeTaskId(bool is_main_task, char * title)
    {
      if (is_main_task)
      {
        sprintf(title,"maintask_%d", rand() % 10000);
      }
      else
      {
        sprintf(title,"subtask_%d", rand() % 10000);
      }
    }

    int requestNewTask(int location_idx)
    {
      gopher_cns_msgs::JobCommand newTaskCmd;
      gopher_cns_msgs::Instr newSubTaskCmd;

      char lc_taskid[100]={0,};
      char lc_subTaskid[100]={0,};
      makeTaskId(true, lc_taskid);
      makeTaskId(false, lc_subTaskid);
      std::vector<std::string> mNullParam;
      std::vector<gopher_cns_msgs::Instr> mSubtask;
      std::vector<std::string> mGoal;
      mNullParam.push_back("");

      switch(location_idx)
      {
        case SILVER_POSITION_HOMEBASE: mGoal.push_back("homebase"); break;
        case SILVER_POSITION_TOILET: mGoal.push_back("target1"); break;
        case SILVER_POSITION_BEDROOM: mGoal.push_back("target2"); break;
        case SILVER_POSITION_LIVING: mGoal.push_back("target3"); break;
        case SILVER_POSITION_SCALE: mGoal.push_back("target4"); break;
        case SILVER_POSITION_TOILET_EMER: mGoal.push_back("target5"); break;
        case SILVER_POSITION_SCALE_EMER: mGoal.push_back("target6"); break;
        // case SILVER_POSITION_SAME: mGoal.push_back("target6"); break;       
        default: mGoal.push_back(""); break;
      }

      ROS_INFO("Requesting New Task id:%s", lc_taskid);

      newSubTaskCmd.job_id = lc_taskid;
      newSubTaskCmd.instr_id = lc_subTaskid;
      newSubTaskCmd.instr_type = 1; // 1 is TYPE_MOVE
      newSubTaskCmd.params = mGoal;
      newSubTaskCmd.created_at = ros::Time::now();
      newSubTaskCmd.modified_at = ros::Time::now();
      newSubTaskCmd.started_at = ros::Time::now();
      newSubTaskCmd.finished_at = ros::Time::now();
      newSubTaskCmd.retried = 0;
      newSubTaskCmd.status = 0;
      mSubtask.push_back(newSubTaskCmd);

      newTaskCmd.request.job.job_id = lc_taskid;
      newTaskCmd.request.job.instrs = mSubtask;
      newTaskCmd.request.job.params = mNullParam;
      newTaskCmd.request.job.job_status = 0;
      newTaskCmd.request.job.job_status_str= "";
      newTaskCmd.request.job.created_at = ros::Time::now();
      newTaskCmd.request.job.modified_at = ros::Time::now();
      newTaskCmd.request.job.started_at = ros::Time::now();
      newTaskCmd.request.job.finished_at = ros::Time::now();
      newTaskCmd.request.job.retasked = 0;

      if(srvTask.call(newTaskCmd))
      {
        srvTask.waitForExistence(ros::Duration(1.0));
      }

      return newTaskCmd.response.result;
    }   

    void iotCallRobotCallback(const silver_msgs::IoTCallRobotConstPtr &iot_msg)
    {
        ROS_INFO("IoT_call_robot_cbk - evt[%d]   pos_id[%d]", iot_msg->iot_event_state, iot_msg->position_id);
        if ( /*iot_msg->iot_event_state == 109 &&*/ sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_CALLROBOT;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotM2MDeviceCallback(const silver_msgs::IoTM2MDeviceConstPtr &iot_msg)
    {
        ROS_INFO("===== IoT M2M Devices Value =====");
        ROS_INFO("iot_motion_status :%d", iot_msg->iot_motion_status);
        ROS_INFO("iot_multi_status  :%d", iot_msg->iot_multi_status);
        ROS_INFO("iot_scale_status  :%d", iot_msg->iot_scale_status);
        ROS_INFO("iot_wearable_status");
        ROS_INFO(" - gyro  [%.2f,  %.2f,   %.2f]", iot_msg->iot_wearable_status[0], iot_msg->iot_wearable_status[1], iot_msg->iot_wearable_status[2]);
        ROS_INFO(" - acc   [%.2f,  %.2f,   %.2f]", iot_msg->iot_wearable_status[3], iot_msg->iot_wearable_status[4], iot_msg->iot_wearable_status[5]);

        nIot_evt = IOT_EVT_NONE;
    }

    void iotEmergCallback(const silver_msgs::IoTEmergStatusConstPtr &iot_emerg_msg)
    {
        ROS_INFO("IoT_emerg_cbk - evt[%d]   pos_id[%d]", iot_emerg_msg->iot_event_state, iot_emerg_msg->position_id);

        if ( ( iot_emerg_msg->iot_event_state == 0)
            || ( iot_emerg_msg->iot_event_state == 1)
            || ( iot_emerg_msg->iot_event_state == 2) ) { // Emergency IoT Event at Toilet
            nIot_evt = IOT_EVT_EMERGENCY;
            nEmerg_evt = iot_emerg_msg->iot_event_state;
            global_position_id = iot_emerg_msg->position_id;
        }
        else {
            nIot_evt = IOT_EVT_NONE;
        }
    }

    void iotEnactCallback(const silver_msgs::IoTEnActConstPtr &iot_msg)
    {
        ROS_INFO("IoT_enact_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 107 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_ENAPP;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotEnappCallback(const silver_msgs::IoTEnAppConstPtr &iot_msg)
    {
        ROS_INFO("IoT_enapp_cbk - evt[%d]   msg_idx[%s]", iot_msg->iot_event_state, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 108 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_ENAPP;
            global_position_id = SILVER_POSITION_LIVING;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotEnWearableCallback(const silver_msgs::IoTEnWearableConstPtr &iot_msg)
    {
        ROS_INFO("IoT_enwearable_cbk - evt[%d]   msg_idx[%s]", iot_msg->iot_event_state, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 110 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_ENWEARABLE;
            global_position_id = SILVER_POSITION_LIVING;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotGoodMorningCallback(const silver_msgs::IoTGoodMorningConstPtr &iot_msg)
    {
        ROS_INFO("IoT_goodmorning_cbk - evt[%d]  type[%d]  msg_idx0[%s]   msg_idx1[%s]   msg_idx2[%s]", \
            iot_msg->iot_event_state, iot_msg->morning_type, iot_msg->msg_idx0.c_str(), iot_msg->msg_idx1.c_str(), iot_msg->msg_idx2.c_str());
        if ( iot_msg->iot_event_state == 101 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_GOODMORNING;
            global_position_id = SILVER_POSITION_BEDROOM;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotGoodNightCallback(const silver_msgs::IoTGoodNightConstPtr &iot_msg)
    {
        ROS_INFO("IoT_goodnight_cbk - evt[%d]   msg_idx[%s]", iot_msg->iot_event_state, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 106 && sequence == SEQ_STANDBY_MODE) {
             nIot_evt = IOT_EVT_GOODNIGHT;
             global_position_id = SILVER_POSITION_BEDROOM;
         }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotHealthReportCallback(const silver_msgs::IoTHealthReportConstPtr &iot_msg)
    {
        ROS_INFO("IoT_health_report_cbk - evt[%d]   pos_id[%d]", iot_msg->iot_event_state, iot_msg->position_id);
        ROS_INFO("IoT_health_report_cbk - update_date[%ld]   health[%d]  life[%d]  rel[%d] rep_type[%d]", \
            iot_msg->update_date, iot_msg->health_indicator, iot_msg->life_indicator, iot_msg->relationship_indicator, iot_msg->report_type);
        if ( iot_msg->iot_event_state == 4 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_HEALTHREPORT;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotTempCareCallback(const silver_msgs::IoTIndoorTempCareConstPtr &iot_msg)
    {
        ROS_INFO("IoT_temp_care_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->indoor_care_msg.c_str());
        if ( iot_msg->iot_event_state == 103 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_INDOORTEMPCARE;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotJustMoveCallback(const silver_msgs::IoTJustMoveConstPtr &iot_move_msg)
    {
        nDeliv_result = 1;
        sDeliv_errMsg = STR_FAIL;
        ROS_INFO("IoT_justmove_cbk - evt[%d]   pos_id[%d]", iot_move_msg->iot_event_state, iot_move_msg->position_id);

        if ( iot_move_msg->iot_event_state == 112 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_JUSTMOVE;
            global_position_id = iot_move_msg->position_id;
            //global_position_id = 0;
        }
        else {
            nIot_evt = IOT_EVT_NONE;
        }

    }
    
    void iotLifePatternCallback(const silver_msgs::IoTLifePatternReportConstPtr &iot_msg)
    {
        ROS_INFO("IoT_life_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 105 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_LIFEPATTERNREPORT;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotMealRecommendCallback(const silver_msgs::IoTMealRecommendConstPtr &iot_msg)
    {
        ROS_INFO("IoT_meal_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        ROS_INFO("IoT_meal_cbk - sky[%d]   fpoisoning[%d]", iot_msg->sky_status, iot_msg->fpoisoning_index);
        if ( iot_msg->iot_event_state == 102 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_MEALRECOMMEND;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotStressCareCallback(const silver_msgs::IoTStressCareConstPtr &iot_msg)
    {
        ROS_INFO("IoT_stress_care_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 104 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_STRESSCARE;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }

    void iotDailyRoutineCallback(const silver_msgs::IoTDailyRoutineConstPtr &iot_msg)
    {
        ROS_INFO("IoT_daily_routine_cbk - evt[%d]   pos_id[%d]  msg_idx[%s]", iot_msg->iot_event_state, iot_msg->position_id, iot_msg->msg_idx.c_str());
        if ( iot_msg->iot_event_state == 132 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_DAILYROUTINE;
            global_position_id = iot_msg->position_id;
        }
        else { nIot_evt = IOT_EVT_NONE; }
    }   

    void iotComeBackCallback(const silver_msgs::RobotStateConstPtr &iot_msg)                              
    {
      ROS_INFO("IoT_comeback_cbk - pos_id[%d]", iot_msg->position_id);                        
      sequence = SEQ_PUB_COMEBACK;
    }
    
    void iotAltoMoveCallback(const silver_msgs::IoTAltoMoveConstPtr &iot_warning_msg)                      
    {
        
   
//if 0 kdh
        if ( iot_warning_msg->iot_event_state == 2 && sequence == SEQ_STANDBY_MODE) {
            nIot_evt = IOT_EVT_WARNING;
            global_position_id = iot_warning_msg->position_id;
        }
        else {
            nIot_evt = IOT_EVT_NONE;
        }
//endif

    }

    void iotSubComeBackBTNCallback(const silver_msgs::ComeBackBTNConstPtr & iot_msg)                          
    {
      c_btn_value = iot_msg->comebackbtn;                       
      publish_comeback_btn(c_btn_value);
    }
    
    void iotReportingCallback(const silver_msgs::IoTWarningStatusConstPtr &iot_warning_msg)                           
    {
      ROS_INFO("IoT_warning_cbk - pos_id[%d]", iot_warning_msg->position_id);
      ROS_INFO("IoT_warning_cbk - s_val[%d]   s_lvl[%d]", iot_warning_msg->stress_value, iot_warning_msg->stress_level);
      ROS_INFO("IoT_warning_cbk - a_val[%d]   a_lvl[%d]", iot_warning_msg->activity_value, iot_warning_msg->activity_level);
      ROS_INFO("IoT_warning_cbk - c_val[%d]   c_lvl[%d]", iot_warning_msg->cardiac_value, iot_warning_msg->cardiac_state);
      ROS_INFO("IoT_warning_cbk - w_val[%d]   b_val[%d]   v_sts[%d]", iot_warning_msg->weight_value, iot_warning_msg->bmi_value, iot_warning_msg->voice_status);
      ROS_INFO("IoT_warning_cbk - s_val_p[%d]   c_val_p[%d]   a_val_p[%d]", iot_warning_msg->stress_value_preferred, iot_warning_msg->cardiac_value_preferred, iot_warning_msg->activity_value_preferred);
    
                               
      id = iot_warning_msg->position_id;                                 
    
      st_value = iot_warning_msg->stress_value;                                 
      st_level = iot_warning_msg->stress_level;                                 
      ac_value = iot_warning_msg->activity_value;                                 
      ac_level = iot_warning_msg->activity_level;                                 
      cd_value = iot_warning_msg->cardiac_value;                                 
      cd_state = iot_warning_msg->cardiac_state;                                 
      wt_vlaue = iot_warning_msg->weight_value;                                 
      b_value = iot_warning_msg->bmi_value;                                 
      v_status = iot_warning_msg->voice_status;
      
      st_value_p = iot_warning_msg->stress_value_preferred;                                 
      cd_value_p = iot_warning_msg->cardiac_value_preferred;                                 
      ac_value_p = iot_warning_msg->activity_value_preferred;                               

      publish_healthdata(id, st_value, st_level, ac_value, ac_level, cd_value, cd_state, wt_vlaue, b_value, v_status, st_value_p, cd_value_p, ac_value_p);
    }
   
    void SkeletonStatusCallback(const silver_msgs::msgSkeletonStatusConstPtr & msg)
    {
        ROS_INFO("Current Skeleton Status : %d", msg->curr_status);
        if ( (msg->curr_status == SKELETON_STATUS_OUT) && (skeleton_auto_finish == true) )
        {
            if(dailyroutine_sequence == SEQ_DAILYROUTINE_SKELETON_TRACKING)
            {
              dailyroutine_sequence = SEQ_DAILYROUTINE_SKELETON_DONE;
            }
            else
            {
              sequence = SEQ_SKELETON_DONE;
            }
           
            if (!greeting_mode)
                syscall_turnoff_skeleton_mode();
        }
        {
            publish_newskeleton();
            ROS_INFO("New Skeleton Person");
        }
    }

    void tabletCallback(const silver_msgs::TabletStatusConstPtr &tablet_msg)
    {
        // Response on Tablet
        ROS_INFO("Tablet_cbk - tbl_status[%d]   tbl_evt[%d]", tablet_msg->tablet_status, tablet_msg->tablet_event);

        // default
        // bUserOk , bModeOff = true

        if ( tablet_msg->tablet_status == 1 && tablet_msg->tablet_event == 1 )              // emerg. checked user 'OK' and comeback : default-false(user not ok)
            bTbl_emerge_userOk = true;
        else if ( tablet_msg->tablet_status == 1 && tablet_msg->tablet_event == 11 )        // emerg. not OK situation !!!
            bTbl_emerge_userOk = false;
        else if ( tablet_msg->tablet_status == 1 && tablet_msg->tablet_event == 2 )         // emerg. require Remote
            bRobot_Control = false;
        else if ( tablet_msg->tablet_status == 1 && tablet_msg->tablet_event == 3 ) {       // emerg. mode off evt : default-false(mode On..)
            //bTbl_emerg_modeOff = true;
            bRobot_Control = true;
        }
        else if ( tablet_msg->tablet_status == 2 && tablet_msg->tablet_event == 22 )        // family(button). out of user
            bTbl_family_userOut = true;
        else if ( tablet_msg->tablet_status == 2 && tablet_msg->tablet_event ==  1 )        // family(button). user OK and then family mode
            bTbl_family_userOk = true;
        else if ( tablet_msg->tablet_status == 2 && tablet_msg->tablet_event == 21 ) {      // family(button). mode off
            bTbl_family_modeOff = true;

        }
        else if ( tablet_msg->tablet_status == 3 && tablet_msg->tablet_event == 31  )        // warning. user is worn
            bTbl_warning_userWearabled = true;
        else if ( tablet_msg->tablet_status == 3 && tablet_msg->tablet_event == 32  )        // warning. user is no worn, time setting in tablet
            bTbl_warning_userWearabled = false;
        else if ( tablet_msg->tablet_status == 3 && tablet_msg->tablet_event == 33  )        // warning. user sleep
            bTbl_warning_userSleep = true;
        else if ( tablet_msg->tablet_status == 3 && tablet_msg->tablet_event == 34 ) {      // warning. mode off
            bTbl_warning_modeOff = true;
        }
        else if ( tablet_msg->tablet_status == 3 && tablet_msg->tablet_event == 0 ) {      // warning. mode off
            nIot_evt = IOT_EVT_WARNING;
            global_position_id = SILVER_POSITION_LIVING;
        }
        else if ( tablet_msg->tablet_status == 4 && tablet_msg->tablet_event == 44 ) {      // final-reporting. mode off
            bTbl_repoting_modeOff = true;
        }
        else if ( tablet_msg->tablet_status == 5 && tablet_msg->tablet_event == 132 ) {
          sequence = SEQ_PUB_SKELETON;
        }
        else if ( tablet_msg->tablet_status == 21 && tablet_msg->tablet_event == 71 )
        {
          dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_GO;
        }
        else if ( tablet_msg->tablet_event == 99 ) {         // esc - comeback home
            //ROS_INFO("esc accepted");
            if (tablet_msg->tablet_status != 15) // To Cover Tablet fault
               bTbl_esc = true;
        }
        else {
            ROS_INFO("Table msg no defined value !!");
        }
    }

    int gotoSomewhere_service_delivery(int curr_position_id)
    {
        nDeliv_result = 1;
        sDeliv_errMsg = STR_FAIL;
             
        requestNewTask(curr_position_id);
        return 0;

        // std::vector<std::string> vs;
       
        // if (curr_position_id == SILVER_POSITION_TOILET) //10 is toilet
        // {
        //     vs.push_back("g_toilet");
        //     //vs.push_back("door");
        // }
        // else if (curr_position_id == SILVER_POSITION_BEDROOM) //20 is bedroom
        // {
        //     vs.push_back("g_bedroom");
        //     //vs.push_back("tv");
        // }
        // else if (curr_position_id == SILVER_POSITION_LIVING) //30 is livingroom
        // {
        //     vs.push_back("g_livingroom");
        //     //vs.push_back("partition");
        // }
        // else if (curr_position_id == SILVER_POSITION_SCALE) // 40 is scale
        // {
        //     vs.push_back("g_scale");
        //     //vs.push_back("inter");
        // }
        // else if (curr_position_id == SILVER_POSITION_TOILET_EMER) // 50 is scale
        // {
        //     vs.push_back("g_toilet_emer");
        //     //vs.push_back("inter");
        // }
        // else if (curr_position_id == SILVER_POSITION_SCALE_EMER) // 60 is scale
        // {
        //     vs.push_back("g_scale_emer");
        //     //vs.push_back("inter");
        // }
        // else if (curr_position_id == SILVER_POSITION_HOMEBASE) // 0 is homebase
        // {
        //     vs.push_back("homebase");
        // }

        // if (curr_position_id != SILVER_POSITION_SAME)
        // {

        //     gopher_delivery_msgs::DeliveryGoal srv;

        //     srv.request.semantic_locations = vs;

        //     if ( srvClient_goal.call(srv) )
        //     {
        //         srvClient_goal.waitForExistence(ros::Duration(1.0));
        //         ROS_INFO("err_msg : %s      result : %d      position : %d", srv.response.error_message.c_str(), srv.response.result, curr_position_id);
        //     }

        //     return srv.response.result;
        // }
        // else
        // {
        //     return 0;
        // }
    }

    void gotoSlip_teleop()
    {
        publish_teleop(0.0, 0.0, 0.0, 0.0, 0.0, 0.2);
    }

    void getList_syscall_delivery()
    {
        int ret = system("gopher_delivery_run -l");
    }

    int syscall_turnoff_obstacle_mode()
    {
        return system("/opt/groot/gopher_ws/devel/silver_long_range_kill_obstacle.sh &");
    }

    int syscall_turnon_obstacle_mode()
    {
        return system("/opt/groot/gopher_ws/devel/silver_long_range_for_obstacle.sh &");
    }

    int syscall_turnoff_skeleton_mode()
    {
        printf("syscall_turnoff_skeleton_mode\n");
        return system("/opt/groot/gopher_ws/devel/silver_long_range_kill_skeleton.sh &");
    }

    int syscall_turnon_skeleton_mode()
    {
        printf("syscall_turnon_skeleton_mode\n");
        return system("/opt/groot/gopher_ws/devel/silver_long_range_for_skeleton.sh &");
    }

    void get_service_deliveryResult()
    {
        return;

        // gopher_delivery_msgs::DeliveryStatus srv;

        // if ( srvClient_result.call(srv) )
        // {
        //     nDeliv_result = srv.response.feedback.status;
        //     sDeliv_errMsg = srv.response.feedback.status_string.c_str();
        //     sDeliv_readableMsg = srv.response.feedback.message.c_str();

        //     ROS_INFO("delivSrvResult[%d], err_msg[%s], reason[%s]", nDeliv_result, sDeliv_errMsg.c_str(), sDeliv_readableMsg.c_str());
        // }
    }

    void publish_robotstatus(u_int8_t ms, bool rc, bool md)
    {
        rs.mode_state = ms;
        rs.robot_control = rc;
        rs.move_done = md;

        pub_robotStatus.publish(rs);
    }
    
    void publish_robotposition(int position)
    {
        rp.position_id = position;
        pub_robotPosition.publish(rp);
    }

    void publish_healthdata(u_int8_t pi, u_int8_t sv, u_int8_t sl, u_int8_t av, u_int8_t al, u_int8_t cv, u_int8_t cs, u_int8_t wv, u_int8_t bv, u_int8_t vs, u_int8_t svp, u_int8_t cvp, u_int8_t avp)          
    {
      hd.position_id = pi;
     
      hd.stress_value = sv;
      hd.stress_level = sl;
      hd.activity_value = av;
      hd.activity_level = al;
      hd.cardiac_value = cv;
      hd.cardiac_state = cs;
      hd.weight_value = wv;
      hd.bmi_value = bv;
      hd.voice_status = vs;
     
      hd.stress_value_preferred = svp;
      hd.cardiac_value_preferred = cvp;
      hd.activity_value_preferred = avp;
     
      pub_reporting.publish(hd);
     
    }
   
    void publish_robotstate(u_int8_t si, u_int8_t pid)                 
    {
      sr.state_id = si;
      sr.position_id = pid;
      pub_robotstate.publish(sr);
    }
    
    void publish_comeback_btn(int cbtnvalue)             
    {
      cbb.comebackbtn = cbtnvalue;
      pub_comeback_btn.publish(cbb);
    }
    
    void publish_teleop(float_t lx, float_t ly, float_t lz, float_t ax, float_t ay, float_t az)
    {
        geometry_msgs::Twist teleop_vel;

        teleop_vel.linear.x = lx;
        teleop_vel.linear.y = ly;
        teleop_vel.linear.z = lz;

        teleop_vel.angular.x = ax;
        teleop_vel.angular.y = ay;
        teleop_vel.angular.z = az;

        pub_gpTeleop.publish(teleop_vel);
    }

    void publish_newskeleton()
    {
        np.check_new_person = 1;
        pub_newPerson.publish(np);
    }

    int dailyroutine(void)
    {
      int ret = 0;
      static int skeleton_count = 0;

      switch(dailyroutine_sequence)
      {
        case SEQ_PUB_DAILYROUTINE:
              ROS_INFO("publishing - master is calling me.. I gotta go");
              publish_robotstatus(21, true, false);
              ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
              dailyroutine_sequence = SEQ_DAILYROUTINE_GO;
              break;

        case SEQ_DAILYROUTINE_GO:
            ROS_INFO("I'm moving~ : %d", global_position_id);
            if (gotoSomewhere_service_delivery(global_position_id) == 0)
                dailyroutine_sequence = SEQ_DAILYROUTINE_GO_FEEDBACK;
            break;

        case SEQ_DAILYROUTINE_GO_FEEDBACK:
            ROS_INFO("master I'm going..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
            get_service_deliveryResult();
            if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
            {
                ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                dailyroutine_sequence = SEQ_DAILYROUTINE_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
            }
            break;

        case SEQ_DAILYROUTINE_GO_DONE:
            ROS_INFO("publishing - go done");
            bTbl_esc = false;
            publish_robotstatus(21, true, true);
            dailyroutine_sequence = SEQ_DAILYROUTINE_MODE;
            break;

        case SEQ_DAILYROUTINE_MODE:
            ROS_INFO("My master I'm here..");
            // checking user OK
            if ( bTbl_esc == true)
            {
                ret = 1;
                dailyroutine_sequence = SEQ_DAILYROUTINE_MAX;
            }
            break;

        /////// SILVER POSITION SCALE ///////
        case SEQ_DAILYROUTINE_SILVER_POSITION_GO:
            ROS_INFO("I'm moving~ : %d", SILVER_POSITION_SCALE);
            if (gotoSomewhere_service_delivery(SILVER_POSITION_SCALE) == 0)
                dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_GO_FEEDBACK;
            break;

        case SEQ_DAILYROUTINE_SILVER_POSITION_GO_FEEDBACK:
            ROS_INFO("master I'm going..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
            get_service_deliveryResult();
            if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
            {
                ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
            }
            break;

        case SEQ_DAILYROUTINE_SILVER_POSITION_GO_DONE:
            ROS_INFO("publishing - go done");
            bTbl_esc = false;
            dailyroutine_sequence = SEQ_DAILYROUTINE_PUB_SKELETON;

            if(skeleton_count > 3)
            {
              skeleton_count = 0;
            }
            syscall_turnoff_obstacle_mode();

            break;

        /////// SKELETON TRACKING ///////
        case SEQ_DAILYROUTINE_PUB_SKELETON:
            ROS_INFO("new tracking skeleton start skeleton_count %d", skeleton_count);
            skeleton_count++;

            if (skeleton_count > 3)
            {
                skeleton_auto_finish = true; //it's real
                syscall_turnon_skeleton_mode();
                // publish_robotposition(SILVER_POSITION_SCALE);
                skeleton_count = 0;
                dailyroutine_sequence = SEQ_DAILYROUTINE_SKELETON_TRACKING;
            }
            break;

        case SEQ_DAILYROUTINE_SKELETON_TRACKING:
            // publish_robotposition(SILVER_POSITION_SCALE);
            if ( nIot_evt == IOT_EVT_EMERGENCY )
            {
                ROS_INFO("This is Emergency Status Finish Tracking");
                nIot_evt = IOT_EVT_NONE;
                syscall_turnoff_skeleton_mode();
                dailyroutine_sequence = SEQ_DAILYROUTINE_SKELETON_DONE_FOR_EMERGENCY;
            }
            else
            {
                ROS_INFO("skeleton tracking...");
            }

            skeleton_count = 0;
            break;

        case SEQ_DAILYROUTINE_SKELETON_DONE_FOR_EMERGENCY:
            ROS_INFO("publishing - tracking skeleton stopped for emergency");
            skeleton_count++;
            if (skeleton_count > 3)
            {
                skeleton_count = 0;
                syscall_turnon_obstacle_mode();
                dailyroutine_sequence = SEQ_PUB_EMERGENCY;
            }
            ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
            skeleton_auto_finish = false;
            break;

        case SEQ_DAILYROUTINE_SKELETON_DONE:
            ROS_INFO("publishing - tracking skeleton stopped");
            //loop_rate(2);
            skeleton_count++;
            if (skeleton_count > 3)
            {
                skeleton_count = 0;
                if (!greeting_mode)
                    syscall_turnon_obstacle_mode();
                dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO;
            }
            ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
            skeleton_auto_finish = false;
            break;           

        /////// SILVER POSITION SCALE ///////
        case SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO:
            ROS_INFO("I'm moving~ : %d", SILVER_POSITION_LIVING);
            if (gotoSomewhere_service_delivery(SILVER_POSITION_LIVING) == 0)
                dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO_FEEDBACK;
            break;

        case SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO_FEEDBACK:
            ROS_INFO("master I'm going...");
            get_service_deliveryResult();
            if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
            {
                ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
            }
            break;

        case SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_GO_DONE:
            ROS_INFO("publishing - go done");
            bTbl_esc = false;
            publish_robotstatus(21, false, false);
            skeleton_count = 0;
            dailyroutine_sequence = SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_MODE;
            break;

        case SEQ_DAILYROUTINE_SILVER_POSITION_LIVING_MODE:
            ROS_INFO("My master I'm here..");
            // checking user OK
            if ( bTbl_esc == true )
            {
                ROS_INFO("ESC : %d", bTbl_esc);
                bTbl_esc = false;
                sequence = SEQ_PUB_COMEBACK;
            }
      }
      return ret;
    }

    void run() {
        //ros::Rate loop_rate(10);         // 2Hz
        ros::Rate loop_rate(2);         // 2Hz
        /*  super-loop for ROS Silver Application  */

        while ( ros::ok() )
        {
            switch ( sequence ) {

                case SEQ_PUB_INIT:
                    ROS_INFO("publishing - init.");
                    publish_robotstatus(11, true, true);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_INIT;
                    if (greeting_mode)
                        syscall_turnoff_obstacle_mode();
                    break;

                case SEQ_INIT:
                    ROS_INFO("Initializing.....");
                    // getList_syscall_delivery();
                    ros::Duration(INIT_SLEEP_TIME_SEC).sleep();
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_PUB_STANDBY;
                    break;

                case SEQ_PUB_STANDBY:
                    ROS_INFO("Publishing - standby");
                    publish_robotstatus(0, true, true);     // robotstatus - 0 : standby
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_STANDBY_MODE;
                    if (greeting_mode)
                        syscall_turnon_skeleton_mode();

                    if (greeting_following_mode)
                    {
                        system("rosnode kill /FollowingCmd");                     
                        system(". /opt/groot/tracker.launch&");
                        sequence = SEQ_GOODMORNING_MODE;
                    }

                    break;

                case SEQ_STANDBY_MODE:
                    if (standby_mode == false)
                    {
                        ROS_INFO("STANDBY... %d", nIot_evt);
                        standby_mode = true;
                    }
                    
                    publish_robotstatus(0, false, false);
            publish_robotstate(SILVER_STATE_READY, SILVER_POSITION_HOMEBASE);                                     
            
                    if ( nIot_evt == IOT_EVT_EMERGENCY ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_EMERGENCY;
                    }
                    else if (nIot_evt == IOT_EVT_CALLROBOT) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_CALLROBOT;
                    }
                    else if (nIot_evt == IOT_EVT_ENACT) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_ENACT;
                    }
                    else if (nIot_evt == IOT_EVT_ENAPP) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_ENAPP;
                    }
                    else if (nIot_evt == IOT_EVT_ENWEARABLE) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_ENWEARABLE;
                    }
                    else if (nIot_evt == IOT_EVT_GOODMORNING) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_GOODMORNING;
                    }
                    else if (nIot_evt == IOT_EVT_GOODNIGHT) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_GOODNIGHT;
                    }
                    else if (nIot_evt == IOT_EVT_HEALTHREPORT) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_HEALTHREPORT;
                    }
                    else if (nIot_evt == IOT_EVT_INDOORTEMPCARE) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_INDOORTEMPCARE;
                    }
                    else if (nIot_evt == IOT_EVT_JUSTMOVE) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_MOVE;
                        //sequence = SEQ_PUB_MOVE_GO_DONE;
                    }
                    else if ( nIot_evt == IOT_EVT_LIFEPATTERNREPORT ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_LIFEPATTERNREPORT;
                    }
                    else if ( nIot_evt == IOT_EVT_MEALRECOMMEND ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_MEALRECOMMEND;
                    }
                    else if ( nIot_evt == IOT_EVT_STRESSCARE ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_STRESSCARE;
                    }
                    else if ( nIot_evt == IOT_EVT_DAILYROUTINE ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_DAILYROUTINE;
                        dailyroutine_sequence = SEQ_PUB_DAILYROUTINE;
                    }
                    else if ( nIot_evt == IOT_EVT_WARNING ) {
                        standby_mode = false;
                        nIot_evt = IOT_EVT_NONE;
                        sequence = SEQ_PUB_WARNING;
                    }

                    break;
/* DAILYROUTINE */
                    case SEQ_DAILYROUTINE:
                      if(dailyroutine() != 0)
                      {
                        ROS_INFO("ESC : %d", bTbl_esc);
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                      }
                    break;
/* DAILYROUTINE */

/* CALL ROBOT */
                    case SEQ_PUB_CALLROBOT:
                        ROS_INFO("publishing - master is calling me.. I gotta go");
                        publish_robotstatus(16, true, false);        // robotstatus - 2 : emergency
                        ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                        sequence = SEQ_CALLROBOT_GO;
                        break;

                    case SEQ_CALLROBOT_GO:
                        ROS_INFO("I'm moving~ : %d", global_position_id);
                        if (gotoSomewhere_service_delivery(global_position_id) == 0)
                            sequence = SEQ_CALLROBOT_GO_FEEDBACK;
                        break;

                    case SEQ_CALLROBOT_GO_FEEDBACK:
                        ROS_INFO("master I'm going..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                        get_service_deliveryResult();
                        if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                        {
                            ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                            sequence = SEQ_PUB_CALLROBOT_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                        }
                        break;

                    case SEQ_PUB_CALLROBOT_GO_DONE:
                        ROS_INFO("publishing - go done");
                        bTbl_esc = false;
                        publish_robotstatus(16, true, true);
                        //publish_test();
                        sequence = SEQ_CALLROBOT_MODE;
                        break;

                    case SEQ_CALLROBOT_MODE:
                        ROS_INFO("My master I'm here..");
                        // checking user OK
                        if ( bTbl_esc == true ) {
                            ROS_INFO("ESC : %d", bTbl_esc);
                            bTbl_esc = false;
                            sequence = SEQ_PUB_COMEBACK;
                        }
                        break;
/* CALL ROBOT */
/* EMERGENCY */
                case SEQ_PUB_EMERGENCY:
                    ROS_INFO("publishing - emergency and go");
                    if (nEmerg_evt == 0)
                    {
                        bTbl_emerge_userOk = false;
                        bTbl_emerg_modeOff = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    else
                    {
                        publish_robotstatus(2, true, false);        // robotstatus - 2 : emergency
                        ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                        sequence = SEQ_EMERGENCY_GO;
                    }
                    break;

                case SEQ_EMERGENCY_GO:
                    ROS_INFO("emergency GO !!!!! : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_EMERGENCY_GO_FEEDBACK;
                    break;

                case SEQ_EMERGENCY_GO_FEEDBACK:
                    ROS_INFO("emgergency going and feedback.....");
                    //ROS_INFO("state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_EMERGENCY_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_EMERGENCY_GO_DONE:
                    ROS_INFO("publishing - go done");
                    bTbl_emerg_modeOff = false;
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(2, true, true);
                    //publish_test();
                    sequence = SEQ_EMERGENCY_MODE;
                    break;

                case SEQ_EMERGENCY_MODE:
                    ROS_INFO("emergency mode... -_-");

                    // checking user OK
                //if ( bTbl_emerge_userOk == true || bTbl_emerg_modeOff == true ) { // User Good condition ^^

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true ) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                        nEmerg_evt = 0;
                    }
                    else if ( bRobot_Control == false ) {
                        bRobot_Control = true;
                        sequence = SEQ_PUB_REMOTE;
                    }
                    else {}
                    break;
/* EMERGENCY */
/* REMOTE CONTROLLING */
                case SEQ_PUB_REMOTE:
                    ROS_INFO("publishing - remote");
                    publish_robotstatus(1, false, true);    // remote mode, controls by Mobile-Tablet
                    sequence = SEQ_REMOTE_MODE;
                    bTbl_esc = false;
                    break;

                case SEQ_REMOTE_MODE:
                    ROS_INFO("remote... ****");
                    // mission completed
                    //if ( bTbl_emerg_modeOff == true || bTbl_esc == true) {
                    if (bTbl_esc == true) {
                        bTbl_emerg_modeOff = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* REMOTE CONTROLLING */
/* ENCOURAGE ACT */
                case SEQ_PUB_ENACT:
                    ROS_INFO("publishing - SEQ_PUB_ENACT");
                    publish_robotstatus(7, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_ENACT_GO;
                    break;

                case SEQ_ENACT_GO:
                    ROS_INFO("SEQ_ENACT_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_ENACT_GO_FEEDBACK;

                    break;

                case SEQ_ENACT_GO_FEEDBACK:
                    ROS_INFO("SEQ_ENACT_GO_FEEDBACK...");//state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_ENACT_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_ENACT_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(7, true, true);
                    sequence = SEQ_ENACT_MODE;
                    break;

                case SEQ_ENACT_MODE:
                    ROS_INFO("SEQ_ENACT_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* ENCOURAGE ACT */
/* ENCOURAGE APPLICATION */
                case SEQ_PUB_ENAPP:
                    ROS_INFO("publishing - SEQ_PUB_ENAPP");
                    publish_robotstatus(14, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_ENAPP_GO;
                    break;

                case SEQ_ENAPP_GO:
                    ROS_INFO("SEQ_ENAPP_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_ENAPP_GO_FEEDBACK;

                    break;

                case SEQ_ENAPP_GO_FEEDBACK:
                    ROS_INFO("SEQ_ENAPP_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_ENPP_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_ENPP_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(14, true, true);
                    sequence = SEQ_ENAPP_MODE;
                    break;

                case SEQ_ENAPP_MODE:
                    ROS_INFO("SEQ_ENAPP_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* ENCOURAGE APPLICATION */
/* ENCOURAGE WEARABLE */
                case SEQ_PUB_ENWEARABLE:
                    ROS_INFO("publishing - SEQ_PUB_ENWEARABLE");
                    publish_robotstatus(18, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_ENWEARABLE_GO;
                    break;

                case SEQ_ENWEARABLE_GO:
                    ROS_INFO("SEQ_ENWEARABLE_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_ENWEARABLE_GO_FEEDBACK;

                    break;

                case SEQ_ENWEARABLE_GO_FEEDBACK:
                    ROS_INFO("SEQ_ENWEARABLE_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_ENWEARABLE_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_ENWEARABLE_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(18, true, true);
                    sequence = SEQ_ENWEARABLE_MODE;
                    break;

                case SEQ_ENWEARABLE_MODE:
                    ROS_INFO("SEQ_ENWEARABLE_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* ENCOURAGE WEARABLE */
/* GOOD MORNING */
                case SEQ_PUB_GOODMORNING:
                    ROS_INFO("publishing - SEQ_PUB_GOODMORNING");
                    publish_robotstatus(8, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_GOODMORNING_GO;
                    system(". /opt/groot/tracker.launch&");
                    break;

                case SEQ_GOODMORNING_GO:
                    ROS_INFO("SEQ_GOODMORNING_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_GOODMORNING_GO_FEEDBACK;

                    break;

                case SEQ_GOODMORNING_GO_FEEDBACK:
                    ROS_INFO("SEQ_GOODMORNING_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_GOODMORNING_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_GOODMORNING_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(8, true, true);
                    sequence = SEQ_GOODMORNING_MODE;
                    break;

                case SEQ_GOODMORNING_MODE:
                    ROS_INFO("SEQ_GOODMORNING_MODE");

                    ROS_INFO("Start srv_following_cmd");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        followingSrv.request.cmd = false;
                        srv_following_cmd.call(followingSrv);
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                        system("rosnode kill /FollowingCmd");
                    }
                    else
                    {
                      if(followingSrv.response.message != "person found")
                      {
                        followingSrv.request.cmd = true;
                        if ( srv_following_cmd.call(followingSrv) )
                        {
                          ROS_INFO("srv_following_cmd is_on : %d",followingSrv.response.is_on);
                          ROS_INFO("srv_following_cmd message : %s",followingSrv.response.message.c_str());
                        }
                        else
                        {
                          ROS_INFO("Call fail!!!!");
                        }
                      }
                      ROS_INFO("End srv_following_cmd");
                    }
                    break;
/* GOOD MORNING */
/* GOOD NIGHT */
                case SEQ_PUB_GOODNIGHT:
                    ROS_INFO("publishing - SEQ_PUB_GOODNIGHT");
                    publish_robotstatus(9, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_GOODNIGHT_GO;
                    break;

                case SEQ_GOODNIGHT_GO:
                    ROS_INFO("SEQ_GOODNIGHT_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_GOODNIGHT_GO_FEEDBACK;

                    break;

                case SEQ_GOODNIGHT_GO_FEEDBACK:
                    ROS_INFO("SEQ_GOODNIGHT_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_GOODNIGHT_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_GOODNIGHT_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(9, true, true);
                    sequence = SEQ_GOODNIGHT_MODE;
                    break;

                case SEQ_GOODNIGHT_MODE:
                    ROS_INFO("SEQ_GOODNIGHT_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* GOOD NIGHT */
/* HEALTH REPORT */
                case SEQ_PUB_HEALTHREPORT:
                    ROS_INFO("publishing - SEQ_PUB_HEALTHREPORT");
                    publish_robotstatus(6, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_HEALTHREPORT_GO;
                    break;

                case SEQ_HEALTHREPORT_GO:
                    ROS_INFO("SEQ_HEALTHREPORT_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_HEALTHREPORT_GO_FEEDBACK;
                    break;

                case SEQ_HEALTHREPORT_GO_FEEDBACK:
                    ROS_INFO("SEQ_HEALTHREPORT_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_HEALTHREPORT_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_HEALTHREPORT_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(6, true, true);
                    sequence = SEQ_HEALTHREPORT_MODE;
                    break;

                case SEQ_HEALTHREPORT_MODE:
                    ROS_INFO("SEQ_HEALTHREPORT_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* HEALTH REPORT */
/* INDOORTEMPCARE */
                case SEQ_PUB_INDOORTEMPCARE:
                    ROS_INFO("SEQ_PUB_INDOORTEMPCARE");
                    publish_robotstatus(10, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();        // robotstatus : 5 - go to wearable device on the bed
                    sequence = SEQ_INDOORTEMPCARE_GO;
                    break;
                case SEQ_INDOORTEMPCARE_GO:
                    ROS_INFO("SEQ_INDOORTEMPCARE_GO to %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_INDOORTEMPCARE_GO_FEEDBACK;

                    break;
                case SEQ_INDOORTEMPCARE_GO_FEEDBACK:
                    ROS_INFO("SEQ_INDOORTEMPCARE_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_INDOORTEMPCARE_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;
                case SEQ_PUB_INDOORTEMPCARE_GO_DONE:
                    ROS_INFO("ARRIVED");
                    publish_robotstatus(10, true, true);
                    sequence = SEQ_INDOORTEMPCARE_MODE;
                    skeleton_count = 0;
                    syscall_turnoff_obstacle_mode();
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    break;
                case SEQ_INDOORTEMPCARE_MODE:
                    ROS_INFO("SEQ_INDOORTEMPCARE_MODE");
                    sequence = SEQ_PUB_SKELETON;
                    skeleton_auto_finish = true; //it's real
                    // skeleton_auto_finish = false; // test code
                    break;
/* INDOORTEMPCARE */
/* SKELETON */
                case SEQ_PUB_SKELETON:
                    ROS_INFO("tracking skeleton start");
                    publish_robotstatus(15, false, false);
                    skeleton_count++;

                    if (skeleton_count > 3)
                    {
                        syscall_turnon_skeleton_mode();
                        publish_robotposition(global_position_id);
                        skeleton_count = 0;
                        sequence = SEQ_SKELETON_TRACKING;
                    }
                    break;

                case SEQ_SKELETON_TRACKING:
                    publish_robotposition(global_position_id);
                    if ( nIot_evt == IOT_EVT_EMERGENCY ) {
                        ROS_INFO("This is Emergency Status Finish Tracking");
                        nIot_evt = IOT_EVT_NONE;
                        skeleton_count = 0;
                        syscall_turnoff_skeleton_mode();
                        sequence = SEQ_SKELETON_DONE_FOR_EMERGENCY;
                    }
                    else
                    {
                        ROS_INFO("skeleton tracking...");
                    }
                    break;

                case SEQ_SKELETON_DONE:
                    ROS_INFO("publishing - tracking skeleton stopped");
                    //loop_rate(2);
                    skeleton_count++;
                    if (skeleton_count > 3)
                    {
                        skeleton_count = 0;
                        if (!greeting_mode)
                            syscall_turnon_obstacle_mode();
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    skeleton_auto_finish = false;
                    break;
                case SEQ_SKELETON_DONE_FOR_EMERGENCY:
                    ROS_INFO("publishing - tracking skeleton stopped for emergency");
                    skeleton_count++;
                    if (skeleton_count > 3)
                    {
                        skeleton_count = 0;
                        syscall_turnon_obstacle_mode();
                        sequence = SEQ_PUB_EMERGENCY;
                    }
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    skeleton_auto_finish = false;
                    break;
/* SKELETON */
/* LIFE PATTERN  REPORT*/
                case SEQ_PUB_LIFEPATTERNREPORT:
                    ROS_INFO("publishing - SEQ_PUB_LIFEPATTERNREPORT");
                    publish_robotstatus(11, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_LIFEPATTERNREPORT_GO;
                    break;

                case SEQ_LIFEPATTERNREPORT_GO:
                    ROS_INFO("SEQ_LIFEPATTERNREPORT_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_LIFEPATTERNREPORT_GO_FEEDBACK;

                    break;

                case SEQ_LIFEPATTERNREPORT_GO_FEEDBACK:
                    ROS_INFO("SEQ_LIFEPATTERNREPORT_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_LIFEPATTERNREPORT_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_LIFEPATTERNREPORT_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(11, true, true);
                    sequence = SEQ_STRESSCARE_MODE;
                    break;

                case SEQ_LIFEPATTERNREPORT_MODE:
                    ROS_INFO("SEQ_LIFEPATTERNREPORT_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* LIFE PATTERN  REPORT*/
/* MEAL RECOMMENT */
                case SEQ_PUB_MEALRECOMMEND:
                    ROS_INFO("SEQ_PUB_MEALRECOMMEND");
                    publish_robotstatus(12, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_MEALRECOMMEND_GO;
                    break;

                case SEQ_MEALRECOMMEND_GO:
                    ROS_INFO("SEQ_MEALRECOMMEND_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_MEALRECOMMEND_GO_FEEDBACK;

                    break;

                case SEQ_MEALRECOMMEND_GO_FEEDBACK:
                    ROS_INFO("SEQ_MEALRECOMMEND_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_MEALRECOMMEND_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_MEALRECOMMEND_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(12, true, true);
                    sequence = SEQ_STRESSCARE_MODE;
                    break;

                case SEQ_MEALRECOMMEND_MODE:
                    ROS_INFO("SEQ_MEALRECOMMEND_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* MEAL RECOMMEND */
/* STRESSCARE */
                case SEQ_PUB_STRESSCARE:
                    ROS_INFO("SEQ_PUB_STRESSCARE");
                    publish_robotstatus(13, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_STRESSCARE_GO;
                    break;

                case SEQ_STRESSCARE_GO:
                    ROS_INFO("SEQ_STRESSCARE_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_STRESSCARE_GO_FEEDBACK;

                    break;

                case SEQ_STRESSCARE_GO_FEEDBACK:
                    ROS_INFO("SEQ_STRESSCARE_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_STRESSCARE_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_STRESSCARE_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_emerge_userOk = false;
                    bTbl_esc = false;
                    publish_robotstatus(13, true, true);
                    sequence = SEQ_STRESSCARE_MODE;
                    break;

                case SEQ_STRESSCARE_MODE:
                    ROS_INFO("SEQ_STRESSCARE_MODE");

                    if ( bTbl_emerge_userOk == true || bTbl_esc == true) { // User Good condition ^^
                        ROS_INFO("UserOK : %d, ESC : %d", bTbl_emerge_userOk, bTbl_esc);
                        bTbl_emerge_userOk = false;
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
/* STRESSCARE */
/* WARNING */
                case SEQ_PUB_WARNING:
                    ROS_INFO("SEQ_PUB_WARNING");
                    publish_robotstatus(5, true, false);
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();        // robotstatus : 5 - go to wearable device on the bed
            publish_robotstate(SILVER_STATE_MOVING,SILVER_POSITION_HOMEBASE);                        
                    sequence = SEQ_WARNING_GO; 
                    break;

                case SEQ_WARNING_GO:
                    ROS_INFO("SEQ_WARNING_GO : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_WARNING_GO_FEEDBACK;

                    break;

                case SEQ_WARNING_GO_FEEDBACK:
                    ROS_INFO("SEQ_WARNING_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
            publish_robotstate(SILVER_STATE_TOHOME, SILVER_POSITION_BEDROOM);       
                        sequence = SEQ_PUB_WARNING_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_WARNING_GO_DONE:
                    ROS_INFO("ARRIVED");
                    bTbl_warning_userWearabled = false;
                    bTbl_warning_userSleep = false;
                    bTbl_warning_modeOff = false;
            publish_robotstate(SILVER_STATE_TOHOME, SILVER_POSITION_BEDROOM);     
                    publish_robotstatus(5, true, true);
//                    sequence = SEQ_STANDBY_MODE;                        
                    bTbl_esc = false;
                    break;

/*                case SEQ_WARNING_MODE:                   
                    ROS_INFO("SEQ_WARNING_MODE");
                    // checking user OK
                    if (bTbl_warning_userWearabled == true    || bTbl_warning_userSleep == true || bTbl_warning_modeOff == true) {
                        ROS_INFO("UserWeared : %d, UserSleep %d, modeOff : %d",
                            bTbl_warning_userWearabled, bTbl_warning_userSleep,    bTbl_warning_modeOff);

                        bTbl_warning_userWearabled = false;
                        bTbl_warning_userSleep = false;
                        bTbl_warning_modeOff = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }

                    if (bTbl_esc == true) {                // esc for comeback.
                        ROS_INFO("esc in warning mode");
                        bTbl_esc = false;
                        sequence = SEQ_PUB_COMEBACK;
                    }
                    break;
*/            
                    /* end of health warning mode */
/* WARNING */
/* JUSTMOVE */
                case SEQ_PUB_MOVE:
                    ROS_INFO("SEQ_PUB_MOVE");
                    publish_robotstatus(17, false, false);        // robotstatus - 2 : emergency
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_MOVE_GO;
                    break;

                case SEQ_MOVE_GO:
                    ROS_INFO("SEQ_MOVE_GO to ID : %d", global_position_id);
                    if (gotoSomewhere_service_delivery(global_position_id) == 0)
                        sequence = SEQ_MOVE_GO_FEEDBACK;

                    break;

                case SEQ_MOVE_GO_FEEDBACK:
                    ROS_INFO("SEQ_MOVE_GO_FEEDBACK..."); //state[%d]     message[%s]", nDelivFeed_state, sDelivFeed_status.c_str());
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("GO DONE -.- delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        sequence = SEQ_PUB_MOVE_GO_DONE;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;

                case SEQ_PUB_MOVE_GO_DONE:
                    ROS_INFO("ARRIVED");
                    publish_robotstatus(17, false, true);
                    skeleton_count = 0;
                    syscall_turnoff_obstacle_mode();
                    ros::Duration(PUB_SLEEP_TIME_SEC).sleep();
                    sequence = SEQ_MOVE_MODE;
                    break;

                case SEQ_MOVE_MODE:
                    ROS_INFO("SEQ_MOVE_MODE");
                    sequence = SEQ_PUB_SKELETON;
                    break;
/* JUSTMOVE */
/* COMEBACK */
                case SEQ_PUB_COMEBACK:
                    ROS_INFO("publishing - comeback");
            publish_robotstate(SILVER_STATE_MOVING, SILVER_POSITION_HOMEBASE);                             
                    publish_robotstatus(3, true, false);
                    sequence = SEQ_COMEBACK;
                    break;

                case SEQ_COMEBACK:
                    ROS_INFO("comeback home... SEOTAEJI");
                    if (gotoSomewhere_service_delivery(SILVER_POSITION_HOMEBASE) == 0)
                        sequence = SEQ_COMEBACK_FEEDBACK;
                    break;

                case SEQ_COMEBACK_FEEDBACK:
                    ROS_INFO("comeback and feedback.....");
                    get_service_deliveryResult();
                    if (( nDeliv_result == 0 ) && (sDeliv_errMsg == "success"))
                    {
                        ROS_INFO("COMEBACK HOME -.- ");
                        ROS_INFO("delivSrvResult[%d], err_msg[%s]", nDeliv_result, sDeliv_errMsg.c_str());
                        publish_robotstatus(3, true, true);
                        sequence = SEQ_PUB_STANDBY;       // 'SUCCESS' is always zero in 'DeliveryErrorCodes.msg'
                    }
                    break;
/* COMEBACK */
                case SEQ_ETC_PROCESS:       // debugging seq.
                    ROS_INFO("debugging sequence - once run");
                    break;

                default:
                    ROS_INFO("sequence exception...");
                    break;
            }

            ros::spinOnce();            // check for callback/service, Handle ROS events
            loop_rate.sleep();
            loop_count++;
        }
    }

private:
    ros::NodeHandle nh;

    ros::Publisher  pub_robotStatus;                            // topic name - '/robot_status', depends on Hierarchy
    ros::Publisher  pub_robotTouch;
    ros::Publisher  pub_newPerson;
    ros::Publisher  pub_gpTeleop;                               // topic '/gopher/yocs_cmd_vel_mux/input/teleop'
    ros::Publisher  pub_Test;
    ros::Publisher  pub_robotPosition;
   
    ros::Publisher  pub_reporting;                               
    ros::Publisher  pub_robotstate;                              
    ros::Publisher  pub_comeback_btn;                            

   
    ros::Subscriber sub_iotStatus_cr;
    ros::Subscriber sub_iotStatus_mm;
    ros::Subscriber sub_iotStatus_gr;
    ros::Subscriber sub_iotStatus_em;                              // subscriber topic '/iot_status/emergency'
    ros::Subscriber sub_iotStatus_ac;
    ros::Subscriber sub_iotStatus_ap;
    ros::Subscriber sub_iotStatus_wb;
    ros::Subscriber sub_iotStatus_gm;
    ros::Subscriber sub_iotStatus_gn;
    ros::Subscriber sub_iotStatus_hr;
    ros::Subscriber sub_iotStatus_tc;
    ros::Subscriber sub_iotStatus_mv;                              // subscriber topic '/iot_status/just_move'
    ros::Subscriber sub_iotStatus_pr;
    ros::Subscriber sub_iotStatus_mr;
    ros::Subscriber sub_iotStatus_sc;
    ros::Subscriber sub_iotStatus_dr;
   
    ros::Subscriber sub_iotStatus_comeback;                             
    ros::Subscriber sub_iotStatus_altomove;                
    ros::Subscriber sub_iotStatus_subcomebackbtn;                       
    ros::Subscriber sub_iotStatus_reporting;                            
   

    ros::Subscriber sub_skeletonStatus;
    ros::Subscriber sub_tabletStatus;                           // subscriber topic '/tablet_status'
    ros::Subscriber sub_tabletConfig;


    ros::Subscriber sub_snd_goal;
    ros::ServiceClient srvTask;   
    // ros::ServiceClient srvClient_goal;                          // service client '/rocon/delivery/goal'
    // ros::ServiceClient srvClient_result;                        // service client '/rocon/delivery/result'

    ros::Subscriber isr_people_tracker_status;
    ros::Subscriber isr_people_tracker_persons;
    ros::Subscriber isr_people_tracker_person;
    ros::ServiceClient srv_following_cmd;

    silver_msgs::RobotMode rs;                                // robot status global message
    silver_msgs::RobotPerson np;
    silver_msgs::VoiceResult vr;
    silver_msgs::RobotPosition rp;
   
    silver_msgs::IoTWarningStatus hd;                              
    silver_msgs::RobotState sr;                                    
    silver_msgs::ComeBackBTN cbb;                                  

   

    u_int8_t loop_count;
    u_int8_t sequence;
    u_int8_t dailyroutine_sequence;
    u_int8_t skeleton_count;
    isr_people_tracker::FollowingCmd followingSrv;

    u_int8_t global_position_id;

    int8_t nDelivFeed_state;
    std::string sDelivFeed_status;

    int8_t nDeliv_result;
    std::string sDeliv_errMsg;
    std::string sDeliv_readableMsg;

    u_int8_t nIot_evt;
    u_int8_t nEmerg_evt;
    
    u_int8_t id;                                       
    u_int8_t st_value;                                 
    u_int8_t st_level;                                
    u_int8_t ac_value;                                 
    u_int8_t ac_level;                                 
    u_int8_t cd_value;                                 
    u_int8_t cd_state;                                 
    u_int8_t wt_vlaue;                                 
    u_int8_t b_value;                                  
    u_int8_t st_value_p;                               
    u_int8_t cd_value_p;                               
    u_int8_t ac_value_p;                               
    u_int8_t v_status;                                 
    u_int8_t c_btn_value;                              
    u_int8_t r_btn_value;                              

    bool    bRobot_Control;                 // True : Robot, False : Tablet
    bool    bTbl_emerge_userOk;             // True : usek ok(^-^), False : emergency situation !
    bool    bTbl_emerg_modeOff;             // True : mode off
    bool    bTbl_family_userOut;            // True : user out, No : user in
    bool    bTbl_family_userOk;             // True : user Ok and Family mode, False : comback
    bool    bTbl_family_modeOff;            // True : mode off
    bool    bTbl_warning_userWearabled;        // True : checked user wearabled device
    bool    bTbl_warning_modeOff;
    bool    bTbl_warning_userSleep;            // True : user is sleep
    bool    bTbl_repoting_modeOff;
    bool    bTbl_esc;                        // True : comeback home only in move_done situation.
    bool    skeleton_auto_finish;
    bool    greeting_mode;
    bool    greeting_following_mode;
    bool    standby_mode;
};



/**
 * This silver ROS application of Emergency Scenario perspective(ESP).
 *
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "silver_application");        // Node
  //  ros::init(argc, argv, "xtionpro_skeleton_pub_node");
    // ros::NodeHandle nh;
    // ros::Publisher xtionpro_skeleton_publisher = nh.advertise<silver_msgs::msgSkeleton>("skeletalpoints", 100);
    // ros::Publisher xtionpro_skeleton_status_publisher = nh.advertise<silver_msgs::msgSkeletonStatus>("skeleton_status", 100);
    // ros::Subscriber sub_position_id = nh.subscribe("/robot_position", 10, subPositionIdCallback);

    SilverApplication sa;

    ROS_INFO("SilverRobot Application Start..... v3.0.2");

    sa.run();

    return 0;
}