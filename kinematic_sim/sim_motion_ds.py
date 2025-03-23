import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'Noto Sans CJK TC']
plt.rcParams["axes.unicode_minus"] = False  # 解决负号显示问题
# ==================== STM32 定时器参数 ====================
system_clock = 250e6          # 系统时钟频率 250 MHz
prescaler = 9                 # 预分频器值 (Prescaler = 10 - 1)
arr = 2499                    # 自动重装载值 (ARR = 2500 - 1)
pwm_frequency = system_clock / ((prescaler + 1) * (arr + 1))  # 计算PWM频率
pwm_period = 1 / pwm_frequency  # PWM周期（秒）

# ==================== 电机与编码器参数 ====================
encoder_ppr = 13              # 编码器线数（脉冲/转）
gear_ratio = 30               # 减速比（30:1）
wheel_diameter = 65e-3        # 车轮直径（米）
wheel_radius = wheel_diameter / 2

# 电机性能参数
no_load_rpm = 366             # 减速后空载转速（RPM）
rated_rpm = 293               # 减速后额定转速（RPM）

# ==================== 仿真参数 ====================
delta_t = 0.1                 # 采样时间间隔（秒）
simulation_time = 10          # 总仿真时间（秒）
num_samples = int(simulation_time / delta_t)

# ==================== 转速计算函数 ====================
def calculate_rpm(delta_pulse, encoder_ppr, gear_ratio, delta_t):
    """根据脉冲变化量计算轮子转速（RPM）"""
    return (delta_pulse * 60) / (encoder_ppr * gear_ratio * delta_t)

# ==================== PWM占空比到电机转速的映射模型 ====================
def pwm_to_rpm(pwm_duty):
    """根据PWM占空比计算理论转速（线性模型，需标定）"""
    # 假设占空比0%对应0 RPM，100%对应空载转速366 RPM
    return no_load_rpm * (pwm_duty / 100)

# ==================== 仿真数据生成 ====================
# 生成时间序列
time = np.arange(0, simulation_time, delta_t)

# 模拟PWM占空比从0%到100%的线性变化
pwm_duty = np.linspace(0, 100, num_samples)

# 根据PWM占空比计算理论转速（含噪声）
target_rpm = pwm_to_rpm(pwm_duty) + np.random.normal(0, 10, num_samples)  # 添加±10 RPM噪声

# 根据目标RPM反推脉冲变化量（含编码器噪声）
delta_pulse = (target_rpm * encoder_ppr * gear_ratio * delta_t) / 60
delta_pulse = np.round(delta_pulse + np.random.normal(0, 2, num_samples)).astype(int)  # 添加±2脉冲噪声

# 计算实际RPM
calculated_rpm = calculate_rpm(delta_pulse, encoder_ppr, gear_ratio, delta_t)

# ==================== 结果可视化 ====================
plt.figure(figsize=(12, 6))

# 绘制PWM占空比与转速关系
plt.subplot(1, 2, 1)
plt.plot(time, pwm_duty, label="PWM占空比 (%)", color='blue')
plt.xlabel("时间 (秒)")
plt.ylabel("占空比 (%)")
plt.title("PWM占空比变化")
plt.grid(True)
plt.legend()

# 绘制目标RPM与实际RPM对比
plt.subplot(1, 2, 2)
plt.plot(time, target_rpm, label="目标RPM", linewidth=2, color='green')
plt.plot(time, calculated_rpm, label="计算RPM（含噪声）", linestyle="--", color='red')
plt.xlabel("时间 (秒)")
plt.ylabel("轮子转速 (RPM)")
plt.title("转速仿真结果（PWM频率={:.1f} kHz）".format(pwm_frequency/1e3))
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()