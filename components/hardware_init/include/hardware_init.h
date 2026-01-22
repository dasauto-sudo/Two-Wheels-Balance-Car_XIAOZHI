#pragma once
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "driver/spi_slave.h"
#include "esp_simplefoc.h"
#include "esp_mac.h"


// 添加BMI270相关头文件
#include "bmi270.h"
#include "bmi2.h"
#include "bmi2_defs.h"

    #if CONFIG_SOC_MCPWM_SUPPORTED
    #define USING_MCPWM
    #endif

    // 引脚定义
    #define MOTOR1_PWM_A      GPIO_NUM_12
    #define MOTOR1_PWM_B      GPIO_NUM_11
    #define MOTOR1_PWM_C      GPIO_NUM_13
    #define MOTOR1_ENABLE     GPIO_NUM_14
    #define MOTOR2_PWM_A      GPIO_NUM_10
    #define MOTOR2_PWM_B      GPIO_NUM_9
    #define MOTOR2_PWM_C      GPIO_NUM_46
    #define MOTOR2_ENABLE     GPIO_NUM_3

    #define I2C0_SDA          GPIO_NUM_4    // BMI270与电机1共享I2C0
    #define I2C0_SCL          GPIO_NUM_5
    #define I2C1_SDA          GPIO_NUM_2
    #define I2C1_SCL          GPIO_NUM_1

    #define SPI_HOST_ID SPI2_HOST
    #define PIN_NUM_MOSI      GPIO_NUM_41
    #define PIN_NUM_MISO      GPIO_NUM_40
    #define PIN_NUM_CLK       GPIO_NUM_39
    #define PIN_NUM_CS        GPIO_NUM_38

    // BMI270 I2C地址
    #define BMI270_I2C_ADDR   0x69
    #define BMI270_INT1       GPIO_NUM_8
    // 电机参数
    #define MOTOR_POLE_PAIRS  7
    #define VOLTAGE_SUPPLY    12.0f
    #define VOLTAGE_LIMIT     6.0f

    #define SVPWM_FREQUENCY   20000

    #define BALANCE_ANGLE_OFFSET 0.35f  // 机械中值（根据实际调整）

    #define LEFT_MOTOR_OFFSET  0.06f     // 左电机死区补偿
    #define RIGHT_MOTOR_OFFSET 0.06f     // 右电机死区补偿
    #define MAX_PWM            60.0f    // 最大PWM限制
    #define FALL_ANGLE         45.0f    // 倾倒角度阈值

    // 卡尔曼滤波器参数
    #define KALMAN_Q_ANGLE    0.001f    // 过程噪声协方差
    #define KALMAN_Q_BIAS     0.003f    // 过程噪声协方差
    #define KALMAN_R_MEASURE  0.03f     // 测量噪声协方差

    // 全局变量声明
    extern float vertical_output;       // 直立环输出
    extern float AngleX;                // X轴角度
    extern float GyroX;                 // X轴角速度
    extern float bias;                  // 角度偏差

    extern float vertical_PWM;          // 直立PWM输出
    extern float speed_PWM;             // 速度PWM输出
    extern float turn_PWM;              // 转向PWM输出
    extern float L_PWM, R_PWM;          // 左右电机PWM
    extern float target_angle_0, target_angle_1; // 目标角度

    // 卡尔曼滤波器结构体
    typedef struct {
        float Q_angle;     // 过程噪声协方差
        float Q_bias;      // 过程噪声协方差
        float R_measure;   // 测量噪声协方差
        
        float angle;       // 角度
        float bias;        // 偏差
        float rate;        // 角速度
        
        float P[2][2];     // 误差协方差矩阵
    } KalmanFilter;

    // BMI270传感器数据结构
    typedef struct {
        float roll;          // X轴角度 (横滚角)
        float pitch;         // Y轴角度 (俯仰角)
        float yaw;           // Z轴角度 (偏航角)
        float angular_vel_x; // X轴角速度 (度/秒)
        float angular_vel_y; // Y轴角速度 (度/秒)
        float angular_vel_z; // Z轴角速度 (度/秒)
        float accel_x;       // X轴加速度 (m/s²)
        float accel_y;       // Y轴加速度 (m/s²)
        float accel_z;       // Z轴加速度 (m/s²)
    } bmi270_data_t;

    typedef struct {
        float Kp;           // 比例系数
        float Ki;           // 积分系数
        float Kd;           // 微分系数
        float outMin;       // 输出最小值
        float outMax;       // 输出最大值
        float setPoint;     // 目标值
        float processValue; // 过程值
        float output;       // 输出值
        float error;        // 误差值
        float lastError;    // 上一次误差
        float integral;     // 积分值
        float derivative;   // 微分值
        float Ki_Out;       // 积分输出
    } PID_Structure;

    typedef struct {
        int16_t throttle;
        int16_t steering;
        uint16_t checksum;
    } control_command_t;

    // 控制状态枚举
    typedef enum {
        CONTROL_STATE_WAITING,    // 等待主机指令
        CONTROL_STATE_RUNNING,    // 正常运行
        CONTROL_STATE_FINISHED,   // 指令执行完毕
        CONTROL_STATE_SAFE        // 安全状态（直立不动）
    } control_state_t;

    // 速度环相关变量
    extern float target_speed;          // 目标速度(rps)
    extern float left_speed, right_speed; // 左右电机实际速度(rps)
    extern float linear_speed, turn_speed;

    // 全局变量声明
    extern BLDCMotor motor1;
    extern BLDCMotor motor2;
    extern BLDCDriver3PWM driver1;
    extern BLDCDriver3PWM driver2;
    extern MT6701 sensor1;
    extern MT6701 sensor2;

    // BMI270相关全局变量
    extern struct bmi2_dev bmi270_dev;
    extern bmi270_data_t bmi270_data;
    extern bool bmi270_calibrated;
    extern KalmanFilter kalmanX, kalmanY, kalmanZ;

    extern i2c_master_bus_handle_t bus_handle0;
    extern i2c_master_bus_handle_t bus_handle1;
    extern i2c_master_dev_handle_t bmi270_dev_handle;

    // BMI270初始化函数声明
    esp_err_t bmi270_init_hardware(void);
    bool bmi270_calibrate_sensors(void);
    void bmi270_update_data(void);
    void kalman_init(KalmanFilter *kalman, float Q_angle, float Q_bias, float R_measure);
    float kalman_update(KalmanFilter *kalman, float newAngle, float newRate, float dt);

// 初始化函数声明
uint16_t calculate_checksum(control_command_t *cmd);
bool validate_command(control_command_t *cmd);
void init_motors();
void init_sensors();
void spi_slave_init(void);
void init_i2c();
void init_imu();
void hardware_init();
void verical_pwm_caculation();
void balance_control_loop();
control_state_t get_control_state();
void set_control_state(control_state_t new_state);
float PID_Adjust_T(PID_Structure *pid, float target, float current, float derivative);