#include "control.h"

/* ══════════════════ Task4 跨圈角度补偿宏(实测微调) ══════════════════ */
/*
 * AC/BD 两段独立补偿:
 *   TASK4_LAP_CALIB_AC_YAW — 作用于 A→C 对角线 (case 0/1/2)
 *   TASK4_LAP_CALIB_BD_YAW — 作用于 B→D 对角线 (case 3/4)
 *   TASK4_LAP_CALIB_BD_YAW_L4_BOOST — BD 第4圈额外追加
 *   TASK4_LAP_CALIB_AC_YAW_L4_BOOST — AC 第4圈额外追加 (叠加在 CALIB_BD*g_lap 之上)
 *
 * 正值(+): 航向目标向外偏(逆时针), 用于修正车偏内侧
 * 负值(-): 航向目标向内偏(顺时针), 用于修正车偏外侧
 * 零值(0): 不补偿, 纯靠初始 g_ref_yaw 基准(同Task3)
 *
 * lap_comp_bd 公式:
 *   g_lap=0~2(第1~3圈): = TASK4_LAP_CALIB_BD_YAW * g_lap
 
 *
 * lap_comp_ac 公式:
 *   g_lap=0~2(第1~3圈): = TASK4_LAP_CALIB_AC_YAW * g_lap
 *   g_lap=3  (第4圈)  : = TASK4_LAP_CALIB_AC_YAW * 3 + TASK4_LAP_CALIB_AC_YAW_L4_BOOST
 *
 * 调参建议: 观察后两圈在 AC 和 BD 哪段对角线偏外更严重,
 *   对应加大该段的负补偿值, 另一段可保持不变或微调.
 *   BD 第4圈非线性漂移：先单独调 BOOST，不动线性基数，避免前几圈跟着跑偏.
 */
#define TASK4_LAP_CALIB_AC_YAW -0.15f
#define TASK4_LAP_CALIB_BD_YAW 0.15f
#define TASK4_LAP_CALIB_BD_YAW_L4_BOOST 0.45f
#define TASK4_LAP_CALIB_AC_YAW_L4_BOOST -1.1f

/* ══════════════════ Task1: A→B 盲跑 ══════════════════ */
void Task1_Loop(void)
{
    if (g_workstep == 0)
    {
        if (Straight_Run())
        { /* B 点 */
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            g_running = false;
            g_task = 0;
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)"Task1 OK", 16);
            OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
        }
    }
}

/* ══════════════════ Task2: A→B→C→D→A ══════════════════ */
void Task2_Loop(void)
{
    switch (g_workstep)
    {
    /* ── A→B 盲走 ── */
    case 0:
        if (Straight_Run())
        {
            Alarm_Beep(); /* B 点声光 */
            g_base_speed = SPEED_ARC;
            g_grace = 5;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 1;
        }
        break;

    /* ── B→C 弧线循迹 ── */
    case 1:
        if (Arc_Run())
        {
            Alarm_Beep(); /* C 点声光 */
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(200);
            angle.target = g_ref_yaw - 180.0f;
            g_base_speed = SPEED_ARC;
            g_grace = 10;
            g_ramp = 30;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(SPEED_ARC, SPEED_ARC);
            g_workstep = 2;
        }
        break;

    /* ── C→D 盲走 ── */
    case 2:
        if (Straight_Run())
        {
            Alarm_Beep(); /* D 点声光 */
            g_ref_yaw = yaw;
            g_base_speed = SPEED_ARC;
            g_grace = 5;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 3;
        }
        break;

    /* ── D→A 弧线循迹 ── */
    case 3:
        if (Arc_Run())
        {
            Alarm_Beep(); /* A 点声光 */
            motor_target_set(0, 0);
            Motor_Brake();
            g_running = false;
            g_task = 0;
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)"Task2 OK", 16);
            OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
        }
        break;
    }
}

/* ══════════════════ Task3: 8字型 A→C→B→D→A (两阶段对角线) ══════════════════ */
/*
 *   每条128cm对角线分 Phase1 + Phase2 两段走
 *   Phase1: 大角度粗略靠岸, 编码器走约100cm退出 (防止角度漂移跑飞)
 *   Phase2: 回正/反转角度, 纯灰度线检测精准停车 (近距离高命中率)
 *   g_ref_yaw 在A点锁一次, 全程不重新锁定 (保持绝对基准防误差叠加)
 */
void Task3_Loop(void)
{
    switch (g_workstep)
    {
    /* ── 初始化: 设第一段对角线角度 ── */
    case 0:
        g_base_speed = SPEED_TASK3;
        angle.target = g_ref_yaw + DIAG_ANGLE_AC1_T3;
        g_grace = 3;
        Kinematics_Clear_Distance();
        PID_Reset_Upper();
        motor_target_set(g_base_speed, g_base_speed);
        g_workstep = 1;
        break;

    /* ── A→C Phase1: 大角度粗略走100cm ── */
    case 1:
        Straight_Run();
        if (Car_Odom.distance_center >= DIAG_DIST_PHASE1_T3)
        {
            PID_Reset_Upper();
            angle.target = g_ref_yaw + DIAG_ANGLE_AC2_T3;
            g_grace = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            g_workstep = 2;
        }
        break;

    /* ── A→C Phase2: 回正方向精确撞线 ── */
    case 2:
        if (Straight_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(200);

            g_base_speed = SPEED_TASK3;
            g_grace = 3;
            g_white_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 3;
        }
        break;

    /* ── C→B 逆时针圆弧循迹 ── */
    case 3:
        if (Arc_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(200);

            g_base_speed = SPEED_TASK3;
            angle.target = g_ref_yaw + DIAG_ANGLE_BD1_T3;
            g_grace = 3;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 4;
        }
        break;

    /* ── B→D Phase1: 大角度粗略走100cm ── */
    case 4:
        Straight_Run();
        if (Car_Odom.distance_center >= DIAG_DIST_PHASE1_T3)
        {
            PID_Reset_Upper();
            angle.target = g_ref_yaw + DIAG_ANGLE_BD2_T3;
            g_grace = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            g_workstep = 5;
        }
        break;

    /* ── B→D Phase2: 反方向精确撞线 ── */
    case 5:
        if (Straight_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(200);

            g_base_speed = SPEED_TASK3;
            g_grace = 3;
            g_white_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 6;
        }
        break;

    /* ── D→A 逆时针圆弧循迹 ── */
    case 6:
        if (Arc_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            g_running = false;
            g_task = 0;
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)"Task3 OK", 16);
            OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
        }
        break;
    }
}

/* ══════════════════ Task4: 8字型 × 4圈 (连续多圈抗扰测试) ══════════════════ */
/*
 * ★ 与旧版的关键区别: g_ref_yaw 锁死在 Task4_Start 时的初始A点值,
 *    跨圈绝不执行 g_ref_yaw = yaw (防止弧线出弯侧滑偏航污染基准).
 *    跨圈漂移由 TASK4_LAP_CALIB_AC_YAW / TASK4_LAP_CALIB_BD_YAW 分AB段线性补偿.
 */
void Task4_Loop(void)
{
    float lap_comp_ac = TASK4_LAP_CALIB_AC_YAW * (float)g_lap;
    if (g_lap >= 3)
        lap_comp_ac += TASK4_LAP_CALIB_AC_YAW_L4_BOOST;
    float lap_comp_bd = TASK4_LAP_CALIB_BD_YAW * (float)g_lap;
    if (g_lap >= 3)
        lap_comp_bd += TASK4_LAP_CALIB_BD_YAW_L4_BOOST;

    switch (g_workstep)
    {
    case 0:
        g_base_speed = SPEED_TASK4;
        angle.target = g_ref_yaw + DIAG_ANGLE_AC1_T4 + lap_comp_ac;
        g_grace = 1;
        g_black_cnt = 0;
        g_white_cnt = 0;
        Kinematics_Clear_Distance();
        PID_Reset_Upper();
        motor_target_set(g_base_speed, g_base_speed);
        g_workstep = 1;
        break;

    case 1:
        Straight_Run();
        if (Car_Odom.distance_center >= DIAG_DIST_PHASE1_AC_T4)
        {
            PID_Reset_Upper();
            angle.target = g_ref_yaw + DIAG_ANGLE_AC2_T4 + lap_comp_ac;
            g_grace = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            g_workstep = 2;
        }
        break;

    case 2:
        if (Straight_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(80);

            g_base_speed = SPEED_TASK4;
            g_grace = 1;
            g_white_cnt = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 3;
        }
        break;

    case 3:
        if (Arc_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(80);

            g_base_speed = SPEED_TASK4;
            angle.target = g_ref_yaw + DIAG_ANGLE_BD1_T4 + lap_comp_bd;
            g_grace = 1;
            g_black_cnt = 0;
            g_white_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 4;
        }
        break;

    case 4:
        Straight_Run();
        if (Car_Odom.distance_center >= DIAG_DIST_PHASE1_BD_T4)
        {
            PID_Reset_Upper();
            angle.target = g_ref_yaw + DIAG_ANGLE_BD2_T4 + lap_comp_bd;
            g_grace = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            g_workstep = 5;
        }
        break;

    case 5:
        if (Straight_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(80);

            g_base_speed = SPEED_TASK4;
            g_grace = 1;
            g_white_cnt = 0;
            g_black_cnt = 0;
            Kinematics_Clear_Distance();
            PID_Reset_Upper();
            motor_target_set(g_base_speed, g_base_speed);
            g_workstep = 6;
        }
        break;

    case 6:
        if (Arc_Run())
        {
            Alarm_Beep();
            motor_target_set(0, 0);
            Motor_Brake();
            delay_ms(80);

            g_lap++;
            if (g_lap >= TOTAL_LAPS)
            {
                Motor_Stop();
                Motor_Brake();
                g_running = false;
                g_task = 0;
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t *)"Task4 OK", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
            else
            {
                g_workstep = 0;
            }
        }
        break;
    }
}
