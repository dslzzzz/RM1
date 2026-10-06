#ifndef PID_H
#define PID_H

// 位置式 PID 控制器
//
// 离散公式:
//   u(k) = Kp*e(k) + Ki*sum(e(j))*T + Kd*D(e(k))
// 其中 T 为采样周期(本工程固定 1kHz -> 0.001s),
// D(.) 是"差分 + 一阶低通"的微分项(见 Pid.cpp):
//   d_raw(k) = (e(k)-e(k-1))/T
//   d(k)     = d(k-1) + alpha*(d_raw(k)-d(k-1)),  alpha = T/(T+tau)
//
// 为什么要给微分加低通: 差分本身等效增益是 1/T = 1000 倍, 编码器只有 8192 计数/圈、
// 转速字段只有 1rpm 分辨率, 直接差分会把量化噪声放大成大电流抖动。加一级 150Hz 低通,
// 在 5~10Hz(本工程速度环的穿越频率)处只损失不到 1° 相位, 高频噪声却能被压下去。
//
// 已内置: 输出限幅 + 抗积分饱和(back-calculation, 用限幅前后的差值反推积分)
class Pid {
public:
    Pid(float kp, float ki, float kd, float out_max, float out_min);
    ~Pid() = default;

    void setParams(float kp, float ki, float kd);
    void setOutputLimit(float out_max, float out_min);
    void reset();

    // 计算一次控制量, ref 目标值, fdb 反馈值, 返回限幅后的输出
    float calc(float ref, float fdb);

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_, ki_, kd_;
    float out_max_, out_min_;
    float integ_;      // 误差累积 sum(e(j))
    float last_err_;   // 上一拍误差 e(k-1)
    float d_filt_;     // 微分项一阶低通的状态(单位: 误差/秒)
    bool  first_;      // 复位后第一拍标志: 不产生微分尖峰
};

#endif // PID_H
