
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG   "motor"
#define LOG_LVL  LOG_LVL_DBG
#include <ulog.h>

#define ENCODE1_NAME   "pulse1"
#define ENCODE2_NAME   "pulse2"


#define MOTOR_LEFT_DIR_PIN    GET_PIN(H, 10)
#define MOTOR_RIGHT_DIR_PIN     GET_PIN(H, 10)

#define ENCODE_PULSE_NUM    (600*4)
#define PULSE_DISTANCE_TRANSFORM   (100)


rt_thread_t motor_speed_query_thread;
rt_thread_t motor_speed_update_thread;

void motor_speed_query_entry(void* param);
void motor_speed_update_entry(void* param);

typedef struct
{
    struct rt_device *encode_dev;

    int16_t real_speed;
    int16_t final_expect_speed;
    int8_t  acceleration;
    int16_t acc_expect_speed;
}robot_motor_t;


static robot_motor_t motor_left;
static robot_motor_t motor_right;


int16_t motor_get_speed(robot_motor_t* motor)
{
    if(motor == RT_NULL)
    {
        LOG_E("motor get speed param err");
        return 0;
    }
    return motor->real_speed;
}

int16_t motor_set_speed(robot_motor_t* motor , int16_t speed , int8_t acc)
{
    if(motor == RT_NULL)
    {
        LOG_E("motor set speed param err");
        return 0;
    }
    motor->final_expect_speed = speed;
    motor->acceleration = acc;

    return motor->real_speed;
}



void motor_init(void)
{
    motor_left.encode_dev = rt_device_find(ENCODE1_NAME);
    motor_right.encode_dev = rt_device_find(ENCODE2_NAME);

    if(motor_left.encode_dev  == RT_NULL || motor_right.encode_dev == RT_NULL)
    {
        LOG_E("encoder not found");
        return;
    }

    if(rt_device_init(motor_left.encode_dev)!= RT_EOK || rt_device_init(motor_right.encode_dev )!= RT_EOK)
    {
        LOG_E("encoder init faild");
        return;
    }
    rt_device_open(motor_left.encode_dev,RT_NULL);
    rt_device_open(motor_right.encode_dev ,RT_NULL);

    rt_pin_mode(MOTOR_LEFT_DIR_PIN  , PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_RIGHT_DIR_PIN , PIN_MODE_OUTPUT);

    motor_speed_query_thread  = rt_thread_create("motor", motor_speed_query_entry , RT_NULL, 512, 1, 10);
    motor_speed_update_thread = rt_thread_create("motor", motor_speed_update_entry, RT_NULL, 512, 2, 10);

    if(motor_speed_query_thread == RT_NULL || motor_speed_update_thread == RT_NULL)
    {
        LOG_E("motor thread create faild");
        return;
    }
    rt_thread_startup(motor_speed_query_thread);
    rt_thread_startup(motor_speed_update_thread);
}

void motor_speed_query_entry(void* param)
{
    int32_t count1 = 0,count2 =0;
    while(1)
    {
        rt_device_read(motor_left.encode_dev, 0, &count1, 1);
        rt_device_read(motor_right.encode_dev, 0, &count2, 1);

        motor_left.real_speed =   count1*PULSE_DISTANCE_TRANSFORM  / ENCODE_PULSE_NUM;
        motor_right.real_speed =  count2*PULSE_DISTANCE_TRANSFORM  / ENCODE_PULSE_NUM;

        LOG_D("motor_left  speed is : %d mm/s " , motor_left.real_speed);
        LOG_D("motor_right speed is : %d mm/s"  ,  motor_right.real_speed );
        rt_device_control(motor_left.encode_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        rt_device_control(motor_right.encode_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);

        rt_thread_mdelay(10);
    }
}

void dac_output(uint8_t ch ,int16_t value)
{
    if(ch == 0)
    {
        rt_pin_write(MOTOR_LEFT_DIR_PIN, value >= 0?  PIN_HIGH:PIN_LOW);
        //set out dac
    }
    else
    {
        rt_pin_write(MOTOR_RIGHT_DIR_PIN, value >= 0?  PIN_HIGH:PIN_LOW);
        //set out dac
    }
}

int16_t cal_pid(int16_t real , int16_t expect)
{
    return 0;
}

void motor_speed_update_entry(void* param)
{
    uint16_t dac_left = 0;
    uint16_t dac_right = 0;

    while(1)
    {
        dac_left  = cal_pid(motor_left.real_speed  ,  motor_left.final_expect_speed);
        dac_right = cal_pid(motor_right.real_speed ,  motor_right.final_expect_speed);

        dac_output(0 , dac_left);
        dac_output(1 , dac_right);
        rt_thread_mdelay(10);
    }
}

void motor_speed_acc_entry(void* param)
{
    while(1)
    {
        if (abs(motor_left.final_expect_speed -  motor_left.acc_expect_speed) > abs(motor_left.acceleration))
        {
            motor_left.acc_expect_speed   +=  motor_left.acceleration;
        }
        else
        {
            motor_left.acc_expect_speed = motor_left.final_expect_speed ;
        }

        if (abs(motor_right.final_expect_speed -  motor_right.acc_expect_speed) > abs(motor_right.acceleration))
        {
            motor_right.acc_expect_speed  +=  motor_right.acceleration;
        }
        else
        {
            motor_right.acc_expect_speed = motor_right.final_expect_speed ;
        }

        rt_thread_mdelay(3);
    }
}



