#include "mpu6050.h"
#include "math.h"

// Hằng số chuyển đổi radian sang độ
#define RAD_TO_DEG 57.29577951308232

// địa chỉ cảm biến
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define SMPLRT_DIV_REG       0x19
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B
#define TEMP_OUT_H_REG       0x41
#define GYRO_CONFIG_REG      0x1B
#define GYRO_XOUT_H_REG      0x43

// Địa chỉ I2C của MPU6050
#define MPU6050_ADDR         0xD0

// Hằng số chia độ nhạy
#define LSB_ACC              16384.0f   // ±2g → 16384 LSB/g
#define LSB_GYRO             131.0f     // ±250 dps → 131 LSB/(°/s)

// Timeout mặc định cho I2C
const uint16_t i2c_timeout = HAL_MAX_DELAY;

// Hệ số hiệu chỉnh trục Z cho cảm biến gia tốc
const double Accel_Z_corrector = 14418.0;

// Biến lưu thời gian lần đo trước
uint32_t timer = 0;

// Cấu hình bộ lọc Kalman cho góc X và Y
Kalman_t KalmanX = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};
Kalman_t KalmanY = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};

/**
 * @brief Khởi tạo MPU6050
 * @retval 0 nếu thành công, 1 nếu lỗi
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check = 0, data = 0;

    // Đọc thanh ghi WHO_AM_I để kiểm tra thiết bị
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);
    if (check != 104) return 1; // 0x68 là ID mặc định của MPU6050

    // Đưa cảm biến ra khỏi chế độ sleep
    data = 0;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, i2c_timeout);

    // Cấu hình tốc độ lấy mẫu (1KHz / (1 + 7) = 125Hz)
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, i2c_timeout);

    // Cấu hình gia tốc kế ±2g
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, i2c_timeout);

    // Cấu hình con quay hồi chuyển ±250°/s
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, i2c_timeout);

    return 0;
}

/**
 * @brief Đọc toàn bộ dữ liệu (gia tốc, nhiệt độ, gyro) và tính góc roll/pitch bằng Kalman
 */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data) {
    uint8_t rec_data[14];

    // Đọc 14 byte từ thanh ghi ACCEL_XOUT_H: ACCEL(6B) + TEMP(2B) + GYRO(6B)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 14, i2c_timeout);

    // Giải mã dữ liệu gia tốc thô
    data->Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    data->Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    data->Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    // Chuyển đổi gia tốc thô sang đơn vị m/s²
    data->Ax = data->Accel_X_RAW * 981.0f / LSB_ACC;
    data->Ay = data->Accel_Y_RAW * 981.0f / LSB_ACC;
    data->Az = data->Accel_Z_RAW * 981.0f / Accel_Z_corrector;

    // Nhiệt độ (không bắt buộc)
    int16_t temp_raw = (int16_t)(rec_data[6] << 8 | rec_data[7]);
    data->Temperature = (float)temp_raw / 340.0f + 36.53f;

    // Giải mã dữ liệu gyro
    data->Gyro_X_RAW = (int16_t)(rec_data[8] << 8 | rec_data[9]);
    data->Gyro_Y_RAW = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    data->Gyro_Z_RAW = (int16_t)(rec_data[12] << 8 | rec_data[13]);

    // Chuyển gyro từ LSB sang độ/giây (°/s)
    data->Gx = data->Gyro_X_RAW / LSB_GYRO;
    data->Gy = data->Gyro_Y_RAW / LSB_GYRO;
    data->Gz = data->Gyro_Z_RAW / LSB_GYRO;

    // Tính thời gian dt giữa hai lần đo
    double dt = (double)(HAL_GetTick() - timer) / 1000.0;
    timer = HAL_GetTick();

    // Tính góc roll từ gia tốc (gần đúng)
    double roll_sqrt = sqrt(data->Accel_X_RAW * data->Accel_X_RAW + data->Accel_Z_RAW * data->Accel_Z_RAW);
    double roll = (roll_sqrt != 0.0) ? atan(data->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG : 0.0;

    // Tính góc pitch từ gia tốc bằng atan2
    double pitch = atan2(-data->Accel_X_RAW, data->Accel_Z_RAW) * RAD_TO_DEG;

    // Bảo vệ bộ lọc Kalman khỏi nhảy đột ngột khi pitch vượt ±90°
    if ((pitch < -90 && data->KalmanAngleY > 90) || (pitch > 90 && data->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        data->KalmanAngleY = pitch;
    } else {
        data->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, data->Gy, dt);
    }

    // Nếu pitch vượt ±90°, đảo chiều trục X
    if (fabs(data->KalmanAngleY) > 90)
        data->Gx = -data->Gx;

    // Cập nhật roll bằng Kalman filter
    data->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, data->Gy, dt);
}

/**
 * @brief Bộ lọc Kalman 1 chiều cho dữ liệu góc
 */
double Kalman_getAngle(Kalman_t *kalman, double newAngle, double newRate, double dt) {
    // Dự đoán bước tiếp theo
    double rate = newRate - kalman->bias;
    kalman->angle += dt * rate;

    // Cập nhật ma trận hiệp phương sai
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Tính độ tin cậy đo (S)
    double S = kalman->P[0][0] + kalman->R_measure;

    // Kalman Gain
    double K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Sai số giữa đo và dự đoán
    double y = newAngle - kalman->angle;

    // Cập nhật góc và bias
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Cập nhật lại ma trận P
    double P00_temp = kalman->P[0][0];
    double P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}
