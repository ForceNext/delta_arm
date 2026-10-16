#include "arm_drive_task.hpp"
#include <cmath>

// 运动参数（来自上位机实测）：加减速 200，速度 1000 RPM
constexpr uint8_t  MOVE_ACCEL = 200;
constexpr uint16_t MOVE_SPEED = 1000;
constexpr uint32_t OFFLINE_FAIL_LIMIT = 20;  // 连续【超时】20 次判定掉线
constexpr uint32_t REPROBE_PERIOD    = 200;  // 停机后每 200 拍（≈1s）重探一次
constexpr uint32_t WAIT_PROBE_PERIOD = 20;   // 等待电机上电时每 20 拍（≈100ms）探测一次

// 手册 5.4.13：0xFC 刹停后需及时 0xFB 清状态，否则电机可能严重发烫。
// 清状态只清"堵转/刹车/失能"标志位，电机仍保持使能与位置闭环。
constexpr bool CLEAR_STATUS_AFTER_BRAKE = true;

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name,
                           int argc, char** argv)
    : PeriodicTask(taskManager, period, name),
      port_(argc, argv),
      master_(port_),
      hw_cfg_((argc > 1) ? argv[1] : "configs/hardware_config.yaml"),
      kin_(Kinematics::fromYaml(hw_cfg_))
{
    master_.resp_timeout_ms = 20;   // 等应答首字节
    // 帧内字节间隔：驱动器在电机运动/忙碌时回帧可能被拆成两段，
    // 5ms 会把中间的正常停顿误判成超时，放宽到 12ms。
    master_.byte_timeout_ms = 12;
    // 发帧前要求总线静默 1ms：6 拍循环里每拍只有 1 次事务，这点开销可承受
    master_.idle_gap_ms = 1;

    int latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
       printf("error: open /dev/cpu_dma_latency\n");
    }
    else
    {
        int latency_target_value = 0;
        int err = write(latency_target_fd, &latency_target_value, 4);
        if(err == -1)
        {
            printf("error: write /dev/cpu_dma_latency\n");
        }
        else
        {
            printf("set /dev/cpu_dma_latency %d\n",latency_target_value);
        }
    }
}

void ArmDriveTask::print_loop_rate()
{
    loop_counter_++;

    if (loop_counter_ == 1) 
    {
        time_start_sec_ = time_ctrl.getSeconds();
        return;
    }
    if (loop_counter_ >= 1000) 
    {
        double now = time_ctrl.getSeconds();
        double duration = now - time_start_sec_;
        double hz = 1000.0 / duration;

        printf_color(PrintColor::Green, "[DriveTask] 1000 loops: %.6f s  ->  %.2f Hz\n",
                duration, hz);

        loop_counter_ = 0;
    }
}

void ArmDriveTask::init()
{
    loop_counter_ = 0;
    time_start_sec_ = 0.0;
    dt = 0.0;
    cycle_ = 0;
    time_ctrl.start();

    // 复位掉线停机 / 等待上电状态
    for (int i = 0; i < 3; ++i) {
        consec_fail_[i] = 0;
        last_rc_[i] = 0;
        nontimeout_err_[i] = 0;
        actual_pos_[i] = 0;
        actual_joint_[i] = 0.0;
        actual_valid_[i] = false;
    }
    emergency_stop_ = false;
    reprobe_counter_ = 0;
    motors_ready_ = false;
    prev_online_mask_ = 0;
    next_motor_ = 1;
    wait_probe_addr_ = 1;
    mb_errcnt_ = 0;
    mb_disp_ = 0;
    mb_read_ = 0;
    cycle_step_ = 0;
    actual_pose_valid_ = false;
    actual_xyz_[0] = actual_xyz_[1] = actual_xyz_[2] = 0.0;

    shm_.connect();  // 连接共享内存：首次创建，其余附加

    // 只读配置里的电机地址；在线探测与使能放到 run()，等全部在线再使能
    MotorMapping::loadMotorAddrs(hw_cfg_);
}

void ArmDriveTask::run()
{
    print_loop_rate();

    ArmCommand cmd;
    shm_.readCommand(cmd);

    double fk_xyz[3] = {cmd.x, cmd.y, cmd.z};   // 回显坐标，默认用目标值

    // ===== 等待电机上电：持续探测，全部在线才使能并转入运行 =====
    if (!motors_ready_) {
        if (++reprobe_counter_ >= WAIT_PROBE_PERIOD) {
            reprobe_counter_ = 0;
            // 每轮只探一台（1→2→3 轮转）：不存在的地址每次要等满 20ms 超时，
            // 一轮探三台会把 200Hz 循环堵住 60ms，是周期掉到 180Hz 的主因。
            int id = wait_probe_addr_;
            wait_probe_addr_ = (wait_probe_addr_ % 3) + 1;
            int rc = MotorMapping::probeMotorRc(master_, id);
            if (rc == MB_RC_OK) {
                MotorMapping::setOnline(id, true);
                consec_fail_[id - 1] = 0;
                last_rc_[id - 1] = 0;
            } else if (rc == MB_RC_TIMEOUT) {
                // 只有"完全无应答"才计入离线判定（避免偶发帧错让电机来回上下线）
                if (++consec_fail_[id - 1] >= OFFLINE_FAIL_LIMIT)
                    MotorMapping::setOnline(id, false);
            } else {
                // CRC 错 / 地址功能码错 / 从站异常码：只统计并打印原因，不判离线
                ++nontimeout_err_[id - 1];
                if (last_rc_[id - 1] != rc) {
                    last_rc_[id - 1] = rc;
                    printf_color(PrintColor::Yellow,
                        "[DriveTask] 电机 %d 探测异常：%s\n", id, modbusRcName(rc));
                }
            }

            uint8_t mask = 0;
            for (int i = 1; i <= 3; ++i)
                if (MotorMapping::isOnline(i)) mask |= (uint8_t)(1u << (i - 1));
            if (mask != prev_online_mask_) {
                prev_online_mask_ = mask;
                printf_color(PrintColor::Yellow,
                    "[DriveTask] 等待电机上电：在线 %d%d%d\n",
                    (mask & 1) ? 1 : 0, (mask & 2) ? 1 : 0, (mask & 4) ? 1 : 0);
            }

            if (MotorMapping::allOnline()) {
                // 初始化：设置协议(modbus) + 设置细分，全部成功才使能
                bool all_init_ok = true;
                for (int i = 1; i <= 3; ++i) {
                    if (!MotorMapping::initMotor(master_, i)) {
                        all_init_ok = false;
                        printf_color(PrintColor::Red,
                            "[DriveTask] 电机 %d 初始化失败，下拍重试\n", i);
                    }
                }
                if (all_init_ok) {
                    MotorMapping::enableMotors(master_);
                    motors_ready_ = true;
                    next_motor_ = 1;
                    printf_color(PrintColor::Green,
                        "[DriveTask] 全部在线并初始化完成，已使能，开始运行\n");
                }
            }
        }
        publishStatus(cmd, false, nullptr, fk_xyz);
        ++cycle_;
        return;
    }

    // ===== 掉线停机状态：不下发，只周期重探离线电机，等人工重启 =====
    if (emergency_stop_) {
        if (++reprobe_counter_ >= REPROBE_PERIOD) {
            reprobe_counter_ = 0;
            // 同样每轮只探一台（轮转），避免 3×20ms 超时把循环堵住
            for (int k = 0; k < 3; ++k) {
                int id = wait_probe_addr_;
                wait_probe_addr_ = (wait_probe_addr_ % 3) + 1;
                if (MotorMapping::isOnline(id)) continue;
                int rc = MotorMapping::probeMotorRc(master_, id);
                if (rc == MB_RC_OK) {
                    if (last_rc_[id - 1] != 0) {   // 只在"首次恢复"时提示，避免每秒刷屏
                        last_rc_[id - 1] = 0;
                        printf_color(PrintColor::Yellow,
                            "[DriveTask] 电机 %d 已恢复通信，请重启程序恢复运行\n", id);
                    }
                } else if (rc == MB_RC_TIMEOUT) {
                    MotorMapping::setOnline(id, false);   // 仍无应答
                } else {
                    ++nontimeout_err_[id - 1];
                    if (last_rc_[id - 1] != rc) {
                        last_rc_[id - 1] = rc;
                        printf_color(PrintColor::Yellow,
                            "[DriveTask] 电机 %d 探测异常：%s\n", id, modbusRcName(rc));
                    }
                }
                break;   // 本轮只探这一台
            }
        }
        publishStatus(cmd, false, nullptr, fk_xyz);
        ++cycle_;
        return;
    }

    // ===== 正常控制 =====
    // 下发方向：计数 = 关节角 × 减速比 × (51200/360)（jointAngleToMotorPos）
    // 回读方向：关节角 = 计数 ÷ (51200/360) ÷ 减速比（motorPosToJointAngle）
    uint32_t counts[3] = {0, 0, 0};
    bool reachable = kin_.kinematics_ik(cmd.x, cmd.y, cmd.z);
    if (reachable) {
        for (int i = 0; i < 3; ++i)
            counts[i] = (uint32_t)std::llround(
                MotorMapping::jointAngleToMotorPos(kin_.theta(i), kin_.gearRatio(i)));
        kin_.kinematics_fk(fk_xyz);
    }

    // ===== 正常控制：6 拍一轮，下发与回读交错 =====
    // 拍号 0 发M1 | 1 读M1 | 2 发M2 | 3 读M2 | 4 发M3 | 5 读M3(→三台实测角齐 → 正解)
    // 每拍只做 1 次 Modbus 事务（下发约 3.2ms / 回读约 2.9ms），都能装进 4ms 周期；
    // 一轮 6 拍 = 24ms，每台下发与回读各约 41.7Hz。
    if (cmd.mode == 1 && reachable) {
        const int id = cycle_step_ / 2 + 1;              // 1..3
        const bool is_read = (cycle_step_ % 2) == 1;

        if (!is_read) {
            // ---------- 下发 ----------
            if (MotorMapping::isOnline(id)) {
                modbus_timer_.start();
                int rc = MotorMapping::moveAbsoluteRc(master_, id, counts[id - 1],
                                                      0, MOVE_ACCEL, MOVE_SPEED);
                double t_ms = modbus_timer_.getMs();
                mb_sum_ms_ += t_ms;
                if (t_ms > mb_max_ms_) mb_max_ms_ = t_ms;
                ++mb_count_; ++mb_disp_;

                if (rc != MB_RC_OK) {
                    handleTxFailure(id, rc, "下发");
                } else {
                    consec_fail_[id - 1] = 0;
                    last_rc_[id - 1] = 0;
                }
            }
        } else {
            // ---------- 回读实测位置 ----------
            if (MotorMapping::isOnline(id)) {
                int32_t pos = 0;
                modbus_timer_.start();
                int rc = MotorMapping::readMotorPosRc(master_, id, pos);
                double t_ms = modbus_timer_.getMs();
                mb_sum_ms_ += t_ms;
                if (t_ms > mb_max_ms_) mb_max_ms_ = t_ms;
                ++mb_count_; ++mb_read_;

                if (rc != MB_RC_OK) {
                    handleTxFailure(id, rc, "回读");
                } else {
                    consec_fail_[id - 1] = 0;
                    last_rc_[id - 1] = 0;
                    actual_pos_[id - 1] = pos;
                    // 计数 → 关节角：÷(51200/360) 再 ÷减速比
                    actual_joint_[id - 1] =
                        MotorMapping::motorPosToJointAngle((double)pos, kin_.gearRatio(id - 1));
                    actual_valid_[id - 1] = true;

                    // 三台实测角齐了 → 用实测关节角做正解，得到真实末端坐标
                    if (actual_valid_[0] && actual_valid_[1] && actual_valid_[2]) {
                        if (kin_.kinematics_fk(actual_joint_, actual_xyz_))
                            actual_pose_valid_ = true;
                    }
                }
            }
        }
        cycle_step_ = (cycle_step_ + 1) % 6;
    } else {
        cycle_step_ = 0;       // 空闲或逆解不可达：本轮作废，恢复命令时从"发M1"重新开始
    }

    // 若刚触发停机：对仍在线电机刹车保持（不失能，避免末端掉落），并清除刹车/故障状态
    if (emergency_stop_) {
        for (int id = 1; id <= 3; ++id) {
            if (!MotorMapping::isOnline(id)) continue;
            MotorMapping::stopMotor(master_, id);
            if (CLEAR_STATUS_AFTER_BRAKE)
                MotorMapping::clearStatus(master_, id);   // 手册要求，避免驱动发热/停在故障态
        }
        printf_color(PrintColor::Red,
            "[DriveTask] 已对在线电机执行刹车保持（不失能）%s\n",
            CLEAR_STATUS_AFTER_BRAKE ? "并清除刹车/故障状态（0xFB）" : "");
    }

    publishStatus(cmd, reachable, counts, fk_xyz);

    ++cycle_;

    // 周期性打印 Modbus 事务耗时统计（每 1000 拍 ≈ 4 s）
    if (++mb_loops_ >= 1000) {
        if (mb_count_ > 0) {
            printf_color(PrintColor::Yellow,
                "[Modbus] 1000 loops: %u 次事务（下发 %u / 回读 %u）| 平均 %.3f ms/次 | 最大 %.3f ms/次 | 每拍合计 %.3f ms | 非超时错误 %u\n",
                mb_count_, mb_disp_, mb_read_, mb_sum_ms_ / mb_count_, mb_max_ms_,
                mb_sum_ms_ / 1000.0, mb_errcnt_);
        } else {
            printf_color(PrintColor::Yellow,
                "[Modbus] 1000 loops: 0 次事务（无电机或 mode!=1）\n");
        }
        mb_sum_ms_ = 0.0;
        mb_max_ms_ = 0.0;
        mb_count_ = 0;
        mb_errcnt_ = 0;
        mb_disp_ = 0;
        mb_read_ = 0;
        mb_loops_ = 0;
    }
}

// 一次事务失败的统一处理：只有"超时（完全无应答）"计入掉线判定，
// CRC 错 / 地址功能码错 / 从站异常码只统计并打印原因（避免偶发帧错让电机来回上下线）。
void ArmDriveTask::handleTxFailure(int id, int rc, const char* what)
{
    ++MotorMapping::errs()[id - 1];
    if (last_rc_[id - 1] != rc) {          // 只有原因变化才打印，避免刷屏
        last_rc_[id - 1] = rc;
        printf_color(PrintColor::Red,
            "[DriveTask] 电机 %d %s失败：%s\n", id, what, modbusRcName(rc));
    }
    if (rc == MB_RC_TIMEOUT) {
        if (++consec_fail_[id - 1] >= OFFLINE_FAIL_LIMIT) {
            MotorMapping::setOnline(id, false);
            printf_color(PrintColor::Red,
                "[DriveTask] 电机 %d 连续无应答 %u 次，判定掉线，停机！\n",
                id, OFFLINE_FAIL_LIMIT);
            emergency_stop_ = true;
        }
    } else {
        ++nontimeout_err_[id - 1];
        ++mb_errcnt_;
    }
}

void ArmDriveTask::publishStatus(const ArmCommand& cmd, bool reachable,
                                 const uint32_t* counts, const double* fk_xyz)
{
    ArmStatus st{};
    st.seq = cycle_;

    // state：bit0=正在按命令运行（原有含义），bit1=本次 theta/motor_pos/x/y/z 来自实测回读
    const bool measured = actual_pose_valid_;
    st.state = (uint8_t)(((cmd.mode == 1 && reachable && !emergency_stop_) ? 0x01 : 0x00)
                       | (measured ? 0x02 : 0x00));

    for (int i = 0; i < 3; ++i) {
        st.online[i] = MotorMapping::isOnline(i + 1) ? 1 : 0;
        if (measured) {
            // 实测：位置用读回的计数，角度由计数换算（等价于 ×(51200/360) 后再除回来）
            st.motor_pos[i] = (double)actual_pos_[i];
            st.theta[i]     = MotorMapping::motorPosToAngle((double)actual_pos_[i]);
        } else if (reachable && counts) {
            // 还没读齐三台实测值：退回"命令值 + 命令角正解"
            st.theta[i]     = kin_.motorTheta(i);
            st.motor_pos[i] = (double)counts[i];
        }
    }

    if (measured) {
        st.x = actual_xyz_[0]; st.y = actual_xyz_[1]; st.z = actual_xyz_[2];
    } else {
        st.x = fk_xyz[0]; st.y = fk_xyz[1]; st.z = fk_xyz[2];
    }
    shm_.writeStatus(st);
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}