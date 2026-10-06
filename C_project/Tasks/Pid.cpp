#include "Pid.h"

namespace {
constexpr float kSampleTime = 0.001f;   // 1kHz 控制周期

// ---- 微分项的一阶低通参数 ------------------------------------------------
// alpha = T/(T+tau), tau = 1/(2*pi*fc)。fc = 150Hz 时 alpha ≈ 0.49:
// 5~10Hz(环路穿越频率)处相位损失 <1°, 而 100Hz 以上的微分噪声被明显压低。
constexpr float kTwoPi         = 6.28318530717959f;
constexpr float kDerivFilterHz = 150.0f;
constexpr float kDerivTau      = 1.0f / (kTwoPi * kDerivFilterHz);
constexpr float kDerivAlpha    = kSampleTime / (kSampleTime + kDerivTau);

// ---- 抗积分饱和的回退时间常数 Tt(s) --------------------------------------
// 输出被限幅时, 超出的部分大约在 Tt 时间内把积分"顶回去"。取 5 个采样周期(5ms):
// 比原先"撤销本拍积分"退饱和更快, 又不会把积分抖掉。
constexpr float kTrackingTime = 5.0f * kSampleTime;
}

Pid::Pid(float kp, float ki, float kd, float out_max, float out_min)
    : kp_(kp), ki_(ki), kd_(kd),
      out_max_(out_max), out_min_(out_min),
      integ_(0.0f), last_err_(0.0f), d_filt_(0.0f), first_(true) {}

void Pid::setParams(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void Pid::setOutputLimit(float out_max, float out_min) {
    out_max_ = out_max; out_min_ = out_min;
}

void Pid::reset() {
    integ_    = 0.0f;
    last_err_ = 0.0f;
    d_filt_   = 0.0f;
    first_    = true;
}

float Pid::calc(float ref, float fdb) {
    const float err = ref - fdb;

    // ---- 微分项: 差分 + 一阶低通 ----
    // 复位后的第一拍不做微分, 否则 d_raw = (err-0)/T 会打出一个尖峰。
    const float d_raw = first_ ? 0.0f : (err - last_err_) / kSampleTime;
    d_filt_ += kDerivAlpha * (d_raw - d_filt_);
    first_   = false;

    const float out_raw = kp_ * err
                        + ki_ * integ_ * kSampleTime
                        + kd_ * d_filt_;

    // 输出限幅
    float out = out_raw;
    if (out > out_max_)      out = out_max_;
    else if (out < out_min_) out = out_min_;

    // ---- 积分累加 + 抗积分饱和(back-calculation) ----
    // 积分项的连续形式 I = Ki*∫e dt, 令
    //     dI/dt = Ki*e + (u_sat - u_raw)/Tt
    // 由 integ_ = sum(e) 反推每一拍的增量:
    //     Δsum(e) = e + (u_sat - u_raw)/(Ki*Tt)
    // 这样输出饱和时积分会被主动"顶回去", 而不是只能停止累积。
    if (ki_ > 0.0f) {
        integ_ += err + (out - out_raw) / (ki_ * kTrackingTime);

        // 数值保护: 积分项本身不超过 2 倍输出限幅, 防止异常工况下状态越滚越大
        const float integ_lim = 2.0f * out_max_ / (ki_ * kSampleTime);
        if (integ_ >  integ_lim) integ_ =  integ_lim;
        if (integ_ < -integ_lim) integ_ = -integ_lim;
    } else {
        integ_ = 0.0f;      // Ki=0(纯 PD)时不保留积分状态
    }

    last_err_ = err;
    return out;
}
