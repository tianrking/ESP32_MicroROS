#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C主控制器的配置参数
#define I2C_MASTER_SCL_IO    22    // I2C主控制器的SCL引脚
#define I2C_MASTER_SDA_IO    21    // I2C主控制器的SDA引脚
#define I2C_MASTER_NUM       I2C_NUM_0   // 使用的I2C端口号
#define I2C_MASTER_FREQ_HZ   100000      // I2C通信频率
#define I2C_MASTER_TX_BUF_DISABLE   0    // 不使用事务缓冲
#define I2C_MASTER_RX_BUF_DISABLE   0    // 不使用接收缓冲

#define JY901S_ADDR          0x50        // JY-901S的I2C地址，根据实际修改
#define ACK_CHECK_EN         0x1              // 启用ACK检查
#define ACK_VAL              0x0              // ACK值
#define NACK_VAL             0x1              // NACK值

static const char *TAG = "JY-901S";

// I2C初始化函数
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
}

// 从JY-901S读取数据的函数
static esp_err_t jy901s_read_data(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (JY901S_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (JY901S_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c

#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
#define q0          0x51
#define q1          0x52
#define q2          0x53
#define q3          0x54
      
#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5		

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};
struct SAcc//加速度
{
	short a[3];
	short T;
};
struct SGyro//角速度
{
	short w[3];
	short T;
};
struct SAngle//角度
{
	short Angle[3];
	short T;
};
struct SMag//磁场输出
{
	short h[3];
	short T;
};

struct SDStatus//端口状态数据输出
{
	short sDStatus[4];
};

struct SPress//气压高度
{
	long lPressure;
	long lAltitude;
};

struct SLonLat//经纬度
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};
struct SQ //四元数
{ short q[4];
};
 
void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t data[16]; // 假设我们要读取16个字节的数据

    while (1) {
        esp_err_t ret = jy901s_read_data(data, sizeof(data)); // 读取数据
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Data read: %.*s", sizeof(data), (char*)data); // 打印数据，根据需要解析
        } else {
            ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 等待一段时间再次读取
    }
}
