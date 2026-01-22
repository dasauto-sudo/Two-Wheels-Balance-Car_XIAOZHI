#include "hardware_init.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h"

static const char *TAG = "HARDWARE_INIT";

// 平衡控制相关全局变量定义
float vertical_output = 0.0f;
float AngleX = 0.0f;
float GyroX = 0.0f;
float bias = 0.0f;

float vertical_PWM = 0.0f;
float speed_PWM = 0.0f;
float turn_PWM = 0.0f;
float L_PWM = 0.0f;
float R_PWM = 0.0f;
float target_angle_0 = 0.0f;
float target_angle_1 = 0.0f;

// 速度环相关全局变量
float speed_left = 0.0f;       // 左电机当前速度 (rad/s)
float speed_right = 0.0f;      // 右电机当前速度 (rad/s)

float linear_speed = 0.0f;
float turn_speed = 0.0f;

float last_angle_left = 0.0f;      // 上一次左电机角度
float last_angle_right = 0.0f;     // 上一次右电机角度

// 速度环，转向环PID初使化：
LowPassFilter lpf_throttle(0.4f);
LowPassFilter lpf_steering(0.4f);

#define MOTOR_MAX_TORQUE 100.0f
// 速度环P.I.D.
#define PID_VEL_P (8.0f)
#define PID_VEL_I (0.20f)
#define PID_VEL_D (0.00f)
// 转向环P.I.D.
#define PID_turn_P (2.0f)
#define PID_turn_I (0.00f)
#define PID_turn_D (0.01f)

PIDController pid_vel(PID_VEL_P, PID_VEL_I, PID_VEL_D, 100000.0f, MOTOR_MAX_TORQUE);
PIDController pid_turn(PID_turn_P, PID_turn_I, PID_turn_D, 100000.0f, MOTOR_MAX_TORQUE/2.0f);

// 直立环初使化：
PID_Structure vertical_pid = {
    .Kp = 15.0f,
    .Ki = 0.01f,
    .Kd = 0.8f,
    .outMin = -1000.0f,
    .outMax = 1000.0f,
    .setPoint = 0.0f,
    .processValue = 0.0f,
    .output = 0.0f,
    .error = 0.0f,
    .lastError = 0.0f,
    .integral = 0.0f,
    .derivative = 0.0f,
    .Ki_Out = 0.0f
};

// I2C总线句柄定义
i2c_master_bus_handle_t bus_handle0 = NULL;
i2c_master_bus_handle_t bus_handle1 = NULL;
i2c_master_dev_handle_t bmi270_dev_handle = NULL;

// 全局变量定义
BLDCMotor motor1(MOTOR_POLE_PAIRS);
BLDCMotor motor2(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver1(MOTOR1_PWM_A, MOTOR1_PWM_B, MOTOR1_PWM_C, MOTOR1_ENABLE);
BLDCDriver3PWM driver2(MOTOR2_PWM_A, MOTOR2_PWM_B, MOTOR2_PWM_C, MOTOR2_ENABLE);
MT6701 sensor1(I2C_NUM_0, I2C0_SCL, I2C0_SDA);
MT6701 sensor2(I2C_NUM_1, I2C1_SCL, I2C1_SDA);

// BMI270相关全局变量
struct bmi2_dev bmi270_dev;
bmi270_data_t bmi270_data;
bool bmi270_calibrated = false;

// 卡尔曼滤波器实例
KalmanFilter kalmanX, kalmanY, kalmanZ;

// 全局变量 - 用于存储从SPI接收的控制命令
control_command_t current_command = {0, 0, 0};
SemaphoreHandle_t command_mutex = NULL;
control_state_t control_state = CONTROL_STATE_WAITING;
uint32_t last_command_time = 0;
#define COMMAND_TIMEOUT_MS 500

// 时间跟踪变量
static uint32_t last_update_time = 0;

uint16_t calculate_checksum(control_command_t *cmd) {
    return (uint16_t)(cmd->throttle + cmd->steering);
}

bool validate_command(control_command_t *cmd) {
    uint16_t calculated_checksum = calculate_checksum(cmd);
    return (cmd->checksum == calculated_checksum);
}

void process_control_command(control_command_t *cmd) {
    if (!validate_command(cmd)) {
        ESP_LOGW(TAG, "Checksum error! Throttle: %d, Steering: %d", 
                cmd->throttle, cmd->steering);
        return;
    }
    
    ESP_LOGI(TAG, "Valid command - Throttle: %d, Steering: %d", 
             cmd->throttle, cmd->steering);
}

// SPI从机初始化
void spi_slave_init(void) {
    esp_err_t ret;
    
    spi_slave_interface_config_t slv_config = {
        .spics_io_num = PIN_NUM_CS,
        .flags = 0,
        .queue_size = 3,
        .mode = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = 0,
        .max_transfer_sz = 0,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_0,
        .intr_flags = 0
    };
    
    ret = spi_slave_initialize(SPI_HOST_ID, &buscfg, &slv_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI Slave initialized");
}

// 卡尔曼滤波器初始化
void kalman_init(KalmanFilter *kalman, float Q_angle, float Q_bias, float R_measure) {
    kalman->Q_angle = Q_angle;
    kalman->Q_bias = Q_bias;
    kalman->R_measure = R_measure;
    
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    kalman->rate = 0.0f;
    
    // 初始化误差协方差矩阵
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// 卡尔曼滤波器更新
float kalman_update(KalmanFilter *kalman, float newAngle, float newRate, float dt) {
    // 预测
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;
    
    // 更新误差协方差矩阵
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;
    
    // 计算卡尔曼增益
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;
    
    // 计算角度和偏差
    float y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;
    
    // 更新误差协方差矩阵
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;
    
    return kalman->angle;
}

// I2C读写函数供BMI270库使用
static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    esp_err_t ret;
    
    ret = i2c_master_transmit_receive(bmi270_dev_handle, 
                                      &reg_addr, 1,
                                      reg_data, len,
                                      -1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C读取失败: 0x%02X, 错误: 0x%x", reg_addr, ret);
        return BMI2_E_COM_FAIL;
    }
    
    return BMI2_OK;
}

static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    esp_err_t ret;
    uint8_t *write_buf = (uint8_t*)malloc(len + 1);
    
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "内存分配失败");
        return BMI2_E_NULL_PTR;
    }
    
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], reg_data, len);
    
    ret = i2c_master_transmit(bmi270_dev_handle, write_buf, len + 1, -1);
    
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C写入失败: 0x%02X, 错误: 0x%x", reg_addr, ret);
        return BMI2_E_COM_FAIL;
    }
    
    return BMI2_OK;
}

// 延迟函数
static void bmi2_delay_us(uint32_t period, void *intf_ptr) {
    uint32_t ms = period / 1000;
    if (ms == 0) ms = 1;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// BMI270传感器初始化
esp_err_t bmi270_init_hardware(void) {
    int8_t rslt;
    
    // 初始化设备结构
    memset(&bmi270_dev, 0, sizeof(bmi270_dev));
    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bmi2_i2c_read;
    bmi270_dev.write = bmi2_i2c_write;
    bmi270_dev.delay_us = bmi2_delay_us;
    bmi270_dev.read_write_len = 30;
    bmi270_dev.intf_ptr = NULL;
    
    // 初始化BMI270
    rslt = bmi270_init(&bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270初始化失败: %d", rslt);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BMI270初始化成功");
    
    // 获取芯片ID
    uint8_t chip_id = 0;
    rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, &bmi270_dev);
    if (rslt == BMI2_OK) {
        ESP_LOGI(TAG, "BMI270芯片ID: 0x%02X", chip_id);
    }
    
    // 软重启
    bmi2_soft_reset(&bmi270_dev);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 禁用省电模式
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi270_dev);
    
    // 启用加速度计和陀螺仪
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    struct bmi2_sens_config config[2];
    
    // 配置加速度计 - 使用硬件滤波
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    config[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;  // 硬件滤波
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;  // 平衡车用2G范围
    
    // 配置陀螺仪 - 使用硬件滤波
    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    config[1].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;  // 硬件滤波
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_250;  // 平衡车用250dps
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    
    // 设置传感器配置
    rslt = bmi2_set_sensor_config(config, 2, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "传感器配置失败: %d", rslt);
        return ESP_FAIL;
    }
    
    // 启用传感器
    rslt = bmi2_sensor_enable(sens_list, 2, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "传感器使能失败: %d", rslt);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BMI270传感器已启用，配置硬件滤波");
    return ESP_OK;
}

// 校准BMI270传感器
bool bmi270_calibrate_sensors(void) {
    int8_t rslt;
    
    ESP_LOGI(TAG, "开始BMI270传感器校准...");
    ESP_LOGI(TAG, "请确保传感器：");
    ESP_LOGI(TAG, "1. Z轴朝下（芯片朝下）");
    ESP_LOGI(TAG, "2. 完全静止");
    ESP_LOGI(TAG, "3. 水平放置");
    
    // 等待3秒确保完全静止
    for (int i = 3; i > 0; i--) {
        ESP_LOGI(TAG, "倒计时: %d秒...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // 1. 陀螺仪校准
    ESP_LOGI(TAG, "陀螺仪校准...");
    rslt = bmi2_perform_gyro_foc(&bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "陀螺仪校准失败: %d", rslt);
        return false;
    }
    ESP_LOGI(TAG, "✅ 陀螺仪校准成功");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 2. 加速度计校准（Z轴朝下）
    ESP_LOGI(TAG, "加速度计校准 (Z轴朝下)...");
    struct bmi2_accel_foc_g_value accel_foc = {
        .x = 0,   // X轴不受重力
        .y = 0,   // Y轴不受重力
        .z = 1,   // Z轴受1g重力
        .sign = 1 // 重力方向与Z轴正方向一致
    };
    
    rslt = bmi2_perform_accel_foc(&accel_foc, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "加速度计校准失败: %d，使用未校准数据", rslt);
    } else {
        ESP_LOGI(TAG, "✅ 加速度计校准成功");
    }
    
    // 3. 启用偏移补偿
    bmi2_set_gyro_offset_comp(BMI2_ENABLE, &bmi270_dev);
    bmi2_set_accel_offset_comp(BMI2_ENABLE, &bmi270_dev);
    
    // 初始化卡尔曼滤波器
    kalman_init(&kalmanX, KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
    kalman_init(&kalmanY, KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
    kalman_init(&kalmanZ, KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
    
    bmi270_calibrated = true;
    ESP_LOGI(TAG, "✅ BMI270传感器校准完成");
    
    return true;
}

// 更新BMI270数据（包含硬件滤波+卡尔曼滤波）
void bmi270_update_data(void) {
    int8_t rslt;
    struct bmi2_sens_data sensor_data;
    static uint32_t last_time = 0;
    
    // 读取传感器数据
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "读取BMI270数据失败: %d", rslt);
        return;
    }
    
    // 获取当前时间，计算时间差
    uint32_t current_time = esp_timer_get_time() / 1000; // ms
    float dt = 0.01f; // 默认时间间隔
    
    if (last_time > 0) {
        dt = (current_time - last_time) / 1000.0f; // 转换为秒
        if (dt > 0.1f) dt = 0.01f; // 限制最大时间间隔
    }
    last_time = current_time;
    
    // 转换加速度数据
    float accel_x = sensor_data.acc.x / 16384.0f;  // ±2g范围
    float accel_y = sensor_data.acc.y / 16384.0f;
    float accel_z = sensor_data.acc.z / 16384.0f;
    
    // 转换为m/s²
    bmi270_data.accel_x = accel_x * 9.80665f;
    bmi270_data.accel_y = accel_y * 9.80665f;
    bmi270_data.accel_z = accel_z * 9.80665f;
    
    // 转换陀螺仪数据
    float gyro_x = sensor_data.gyr.x / 16.4f;  // ±250dps范围
    float gyro_y = sensor_data.gyr.y / 16.4f;
    float gyro_z = sensor_data.gyr.z / 16.4f;
    
    // 从加速度计计算角度
    float accel_angle_x = atan2f(accel_y, accel_z) * 180.0f / M_PI;
    float accel_angle_y = atan2f(-accel_x, sqrtf(accel_y*accel_y + accel_z*accel_z)) * 180.0f / M_PI;
    float accel_angle_z = 0.0f; // 加速度计无法提供偏航角
    
    // 应用卡尔曼滤波
    bmi270_data.roll = kalman_update(&kalmanX, accel_angle_x, gyro_x, dt);
    bmi270_data.pitch = kalman_update(&kalmanY, accel_angle_y, gyro_y, dt);
    bmi270_data.yaw = kalman_update(&kalmanZ, bmi270_data.yaw, gyro_z, dt); // 纯陀螺仪积分
    
    // 更新角速度
    bmi270_data.angular_vel_x = gyro_x;
    bmi270_data.angular_vel_y = gyro_y;
    bmi270_data.angular_vel_z = gyro_z;
    
    // 更新全局变量用于直立环计算
    AngleX = bmi270_data.roll;      // 使用X轴（横滚角）作为平衡轴
    GyroX = bmi270_data.angular_vel_x;
    
    // 调试信息（可选）
    static uint32_t last_print_time = 0;
    if (current_time - last_print_time > 1000) { // 每秒打印一次
        ESP_LOGD(TAG, "BMI270 - X:%.2f°, Y:%.2f°, Z:%.2f°, GX:%.2f°/s", 
                bmi270_data.roll, bmi270_data.pitch, bmi270_data.yaw,
                bmi270_data.angular_vel_x);
        last_print_time = current_time;
    }
}

// 电机初始化
void init_motors() {
    // 链接传感器和驱动器
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    // 驱动器配置
    driver1.voltage_power_supply = VOLTAGE_SUPPLY;
    driver1.voltage_limit = VOLTAGE_LIMIT;
    driver2.voltage_power_supply = VOLTAGE_SUPPLY;
    driver2.voltage_limit = VOLTAGE_LIMIT;

#ifdef USING_MCPWM
    driver1.pwm_frequency = SVPWM_FREQUENCY;
    driver2.pwm_frequency = SVPWM_FREQUENCY;
    
    driver1.init(0);
    driver2.init(1);
#else
    driver1.init({1, 2, 3});
    driver2.init({4, 5, 6});
#endif

    // 电机参数配置
    motor1.phase_resistance = 2.9;
    motor1.phase_inductance = 0.0028;
    motor1.KV_rating = 220;
    motor1.voltage_sensor_align = 3;
    motor1.current_limit = 1.0f;

    motor2.phase_resistance = 2.9;
    motor2.phase_inductance = 0.0028;
    motor2.KV_rating = 220;
    motor2.voltage_sensor_align = 3;
    motor2.current_limit = 1.0f;

    // 控制参数
    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;
    
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

    motor1.voltage_limit = VOLTAGE_LIMIT;
    motor2.voltage_limit = VOLTAGE_LIMIT;
    
    motor1.PID_velocity.output_ramp = 10000.0f;
    motor2.PID_velocity.output_ramp = 10000.0f;
    
    motor1.PID_velocity.P = 0.0f;
    motor1.PID_velocity.I = 0.0f;
    motor1.PID_velocity.D = 0.0f;
    motor1.LPF_velocity.Tf = 0.01f;
    
    motor2.PID_velocity.P = 0.0f;
    motor2.PID_velocity.I = 0.0f;
    motor2.PID_velocity.D = 0.0f;
    motor2.LPF_velocity.Tf = 0.01f;

    // 初始化FOC
    motor1.init();
    motor2.init();
    motor1.initFOC();
    motor2.initFOC();
    ESP_LOGI("MOTOR_INIT", "Motors initialized");
}

// 传感器初始化
void init_sensors() {
    sensor1.init();
    sensor2.init();
}

void init_i2c() {
    // I2C0总线初始化 (共享BMI270和电机1)
    i2c_master_bus_config_t bus_cfg0 = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C0_SDA,
        .scl_io_num = I2C0_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = false,
            .allow_pd = false
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg0, &bus_handle0));

    // 为BMI270添加设备到I2C0总线
    i2c_device_config_t bmi270_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle0, &bmi270_config, &bmi270_dev_handle));

    // I2C1总线初始化 (电机2)
    i2c_master_bus_config_t bus_cfg1 = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = I2C1_SDA,
        .scl_io_num = I2C1_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = false,
            .allow_pd = false
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg1, &bus_handle1));

    ESP_LOGI("I2C_INIT", "I2C buses initialized with BMI270 on I2C0");
}

// IMU初始化（现在使用BMI270）
void init_imu() {
    // 初始化BMI270硬件
    if (bmi270_init_hardware() != ESP_OK) {
        ESP_LOGE(TAG, "BMI270硬件初始化失败");
        return;
    }
    
    // 校准传感器
    if (!bmi270_calibrate_sensors()) {
        ESP_LOGW(TAG, "BMI270校准失败，使用未校准数据");
    }
    
    // 初始更新一次数据
    bmi270_update_data();
    
    ESP_LOGI(TAG, "BMI270 IMU初始化完成");
}

// 计算电机转速 (rad/s)
void calculate_motor_speed() {
    speed_left = motor1.shaft_velocity;
    speed_right = motor2.shaft_velocity;
    
    linear_speed = (speed_left + speed_right) / 2.0f;
    turn_speed = (speed_left - speed_right) / 2.0f;
}

// 直立PWM计算（使用BMI270的X轴数据）
void verical_pwm_caculation() {
    // 使用BMI270滤波后的X轴角度和角速度
    float current_angle = bmi270_data.roll;       // X轴角度
    float current_gyro = bmi270_data.angular_vel_x; // X轴角速度
    
    // 对X轴数据取反（因为传感器方向与MPU6050的方向相反）
    current_angle = -current_angle;
    current_gyro = -current_gyro;

    AngleX = current_angle;
    GyroX = current_gyro;
    
    // PID计算
    vertical_PWM = PID_Adjust_T(&vertical_pid, BALANCE_ANGLE_OFFSET, 
                                current_angle, current_gyro);
}

// SPI接收任务
void spi_receive_task(void *pvParameters) {
    esp_err_t ret;
    control_command_t rx_cmd;
    
    spi_slave_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = sizeof(control_command_t) * 8;
    trans.rx_buffer = &rx_cmd;
    trans.tx_buffer = NULL;
    
    while (1) {
        ret = spi_slave_transmit(SPI_HOST_ID, &trans, portMAX_DELAY);
        
        if (ret == ESP_OK && validate_command(&rx_cmd)) {
            if (xSemaphoreTake(command_mutex, portMAX_DELAY) == pdTRUE) {
                current_command = rx_cmd;
                last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                
                if (current_command.throttle == 0 && current_command.steering == 0) {
                    control_state = CONTROL_STATE_FINISHED;
                } else {
                    control_state = CONTROL_STATE_RUNNING;
                }
                
                xSemaphoreGive(command_mutex);
                
                ESP_LOGI(TAG, "Command received - State: %d, Throttle: %d, Steering: %d", 
                        control_state, current_command.throttle, current_command.steering);
            }
        }
        
        memset(&rx_cmd, 0, sizeof(rx_cmd));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 获取当前控制命令
void get_current_command(control_command_t *cmd) {
    if (xSemaphoreTake(command_mutex, portMAX_DELAY) == pdTRUE) {
        *cmd = current_command;
        xSemaphoreGive(command_mutex);
    }
}

// 检查命令超时
void check_command_timeout() {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_command_time > COMMAND_TIMEOUT_MS) {
        control_state = CONTROL_STATE_WAITING;
        if (xSemaphoreTake(command_mutex, portMAX_DELAY) == pdTRUE) {
            current_command.throttle = 0;
            current_command.steering = 0;
            xSemaphoreGive(command_mutex);
        }
    }
}

// 平衡控制主循环
void balance_control_loop() {
    control_command_t cmd;
    
    // 检查命令超时
    check_command_timeout();
    
    // 获取当前命令
    get_current_command(&cmd);
    
    float throttle = 0.0f;
    float steering = 0.0f;
    
    // 根据状态决定控制行为
    switch (control_state) {
        case CONTROL_STATE_WAITING:
            throttle = 0.0f;
            steering = 0.0f;
            ESP_LOGD(TAG, "State: WAITING");
            break;
            
        case CONTROL_STATE_RUNNING:
            throttle = (float)cmd.throttle / 1000.0f;
            steering = (float)cmd.steering / 1000.0f;
            ESP_LOGD(TAG, "State: RUNNING");
            break;
            
        case CONTROL_STATE_FINISHED:
            throttle = 0.0f;
            steering = 0.0f;
            ESP_LOGD(TAG, "State: FINISHED");
            break;
            
        case CONTROL_STATE_SAFE:
        default:
            throttle = 0.0f;
            steering = 0.0f;
            motor1.move(0);
            motor2.move(0);
            ESP_LOGW(TAG, "State: SAFE - Emergency stop");
            return;
    }

    // 更新BMI270传感器数据（包含硬件滤波+卡尔曼滤波）
    bmi270_update_data();
    
    // 更新电机传感器数据
    sensor1.update();
    sensor2.update();
    
    // 计算电机速度
    calculate_motor_speed();
    
    // 计算直立PWM
    verical_pwm_caculation();
    
    // 速度环和转向环计算
    if (control_state != CONTROL_STATE_SAFE) {
        float target_speed = lpf_throttle(throttle) * 100.0f;
        speed_PWM = pid_vel(target_speed - linear_speed);
        
        float target_turn = lpf_steering(steering) * 100.0f;
        turn_PWM = pid_turn(target_turn - turn_speed);
    } else {
        speed_PWM = 0.0f;
        turn_PWM = 0.0f;
    }
    
    // 电机控制逻辑
    float PWM = vertical_PWM + speed_PWM;
    
    // PWM限幅
    if (PWM > MAX_PWM) PWM = MAX_PWM;
    if (PWM < -MAX_PWM) PWM = -MAX_PWM;
    
    L_PWM = PWM - turn_PWM;
    R_PWM = PWM + turn_PWM;
    
    // 死区补偿
    if (PWM > 0) {
        L_PWM = L_PWM + LEFT_MOTOR_OFFSET;
        R_PWM = R_PWM + RIGHT_MOTOR_OFFSET;
    } else if (PWM < 0) {
        L_PWM = L_PWM - LEFT_MOTOR_OFFSET;
        R_PWM = R_PWM - RIGHT_MOTOR_OFFSET;
    } else {
        L_PWM = 0;
        R_PWM = 0;
    }
    
    // 进一步限幅
    L_PWM = (L_PWM > 200.0f) ? 200.0f : (L_PWM < -200.0f) ? -200.0f : L_PWM;
    R_PWM = (R_PWM > 200.0f) ? 200.0f : (R_PWM < -200.0f) ? -200.0f : R_PWM;
    
    // 倾倒检测（使用BMI270的X轴角度）
    // 倾倒检测（使用调整后的X轴角度）- 修改这里
    float current_angle = -bmi270_data.roll;  // 使用取反后的角度
    if (current_angle > FALL_ANGLE || current_angle < -FALL_ANGLE) {
        control_state = CONTROL_STATE_SAFE;
        motor1.move(0);
        motor2.move(0);
        ESP_LOGE(TAG, "Fall detected! AngleX=%.2f°, Entering SAFE state", current_angle);
    } else if (control_state != CONTROL_STATE_SAFE) {
        // 转换为弧度并控制电机
        target_angle_0 = L_PWM/180*M_PI;
        target_angle_1 = R_PWM/180*M_PI;
        
        motor1.move(target_angle_0);
        motor2.move(target_angle_1);
    }
    
    // 运行FOC
    motor1.loopFOC();
    motor2.loopFOC();
}

// 硬件初始化
void hardware_init() {
    // 创建互斥锁
    command_mutex = xSemaphoreCreateMutex();
    if (command_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // 初始化I2C总线（包含BMI270）
    init_i2c();
    
    // 初始化SPI从机
    spi_slave_init();
    
    // 创建SPI接收任务
    xTaskCreatePinnedToCore(spi_receive_task, "spi_receive", 4096, NULL, 5, NULL, 0);
    
    // 初始化其他硬件
    init_sensors();
    init_motors();
    init_imu();
    
    ESP_LOGI(TAG, "Hardware initialization complete with BMI270");
}

// 状态获取函数
control_state_t get_control_state() {
    return control_state;
}

// 手动状态设置函数
void set_control_state(control_state_t new_state) {
    if (xSemaphoreTake(command_mutex, portMAX_DELAY) == pdTRUE) {
        control_state = new_state;
        xSemaphoreGive(command_mutex);
    }
}

// PID计算函数（带微分先行）
float PID_Adjust_T(PID_Structure *pid, float target, float current, float derivative) {
    pid->setPoint = target;
    pid->processValue = current;
    pid->error = pid->processValue - pid->setPoint;
    
    float proportional = pid->Kp * pid->error;
    
    pid->Ki_Out += pid->error;
    pid->Ki_Out = _constrain(pid->Ki_Out, pid->outMin, pid->outMax);
    
    float differential = pid->Kd * derivative;
    
    pid->output = proportional + (pid->Ki * pid->Ki_Out) + differential;
    
    return pid->output;
}

// 添加手动校准函数（用于调试）
void bmi270_manual_calibration(void) {
    ESP_LOGI(TAG, "手动触发BMI270校准...");
    if (bmi270_calibrate_sensors()) {
        ESP_LOGI(TAG, "手动校准成功");
    } else {
        ESP_LOGE(TAG, "手动校准失败");
    }
}

// 获取BMI270数据（用于调试）
bmi270_data_t get_bmi270_data(void) {
    return bmi270_data;
}