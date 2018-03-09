#include "motor_driver.h"
#include "board.h"
#include <errno.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define MODE        PWM_LEFT

#ifdef QDEC_NUMOF
#define SIMU_ENC_BUFSIZE    3
extern int32_t qdecs_value[QDEC_NUMOF];
static void simulate_encoders(uint8_t motor_id, \
    motor_direction_t direction, uint16_t pwm_duty_cycle);

static void simulate_encoders(uint8_t motor_id, \
    motor_direction_t direction, uint16_t pwm_duty_cycle)
{
    static int16_t simu_motor_encoder[QDEC_NUMOF][SIMU_ENC_BUFSIZE] = {0,};

    int16_t i = 0, s = 0;
    int32_t pwm_value = 0;

    pwm_value = direction ? -pwm_duty_cycle : pwm_duty_cycle;

    for (i = 0; i < SIMU_ENC_BUFSIZE - 1; i++)
    {
        s += simu_motor_encoder[motor_id][i];
        simu_motor_encoder[motor_id][i] = simu_motor_encoder[motor_id][i+1];
    }
    s = (s + simu_motor_encoder[motor_id][i] + pwm_value) / (SIMU_ENC_BUFSIZE + 1);

    simu_motor_encoder[motor_id][i] = s;

    qdecs_value[motor_id] = s;

    DEBUG("MOTOR_ID = %u    PWM_VALUE = %d      QDEC_AVERAGE = %d\n", \
        motor_id, pwm_value, s);
}
#endif /* QDEC_NUMOF */


int motor_driver_init(motor_driver_t motor_driver)
{
    if (motor_driver >= MOTOR_DRIVER_NUMOF)
    {
        errno = ENODEV;
        goto motor_init_err;
    }

    motor_driver_config_t motor_driver_conf = motor_driver_config[motor_driver];

    pwm_t pwm_dev = motor_driver_conf.pwm_dev;
    uint32_t freq = motor_driver_conf.pwm_frequency;
    uint16_t resol = motor_driver_conf.pwm_resolution;
    if (pwm_init(pwm_dev, MODE, freq, resol) != freq)
    {
        errno = EINVAL;
        goto motor_init_err;
    }

    return 0;

motor_init_err:
    return -1;
}

int motor_set(const motor_driver_t motor_driver, uint8_t motor_id, \
    motor_direction_t direction, uint16_t pwm_duty_cycle)
{
    const motor_driver_config_t motor_driver_conf = motor_driver_config[motor_driver];
    const motor_t *dev = &motor_driver_conf.motors[motor_id];

    direction = direction ^ dev->gpio_dir_reverse;

    /* Two direction GPIO, handling brake */
    if (motor_driver_conf.mode == MOTOR_DRIVER_2_DIRS)
    {
        switch (direction)
        {
            case MOTOR_CW:
            case MOTOR_CCW:
                /* Direction */
                gpio_write(dev->gpio_dir0, direction);
                gpio_write(dev->gpio_dir1, (direction) ^ 0x1);
                break;
            case MOTOR_BRAKE:
            default:
                /* Brake */
                gpio_write(dev->gpio_dir0, 0);
                gpio_write(dev->gpio_dir1, 0);
                pwm_duty_cycle = 0;
                break;
        }
    }
    /* Single direction GPIO */
    else if (motor_driver_conf.mode == MOTOR_DRIVER_1_DIR)
    {
        switch (direction)
        {
            case MOTOR_CW:
            case MOTOR_CCW:
                /* Direction */
                gpio_write(dev->gpio_dir0, direction);
                break;

            case MOTOR_BRAKE:
            default:
                pwm_duty_cycle = 0;
                break;
        }
    }
    /* Single direction GPIO and brake GPIO */
    else if (motor_driver_conf.mode == MOTOR_DRIVER_1_DIR_BRAKE)
    {
        switch (direction)
        {
            case MOTOR_CW:
            case MOTOR_CCW:
                /* Direction */
                gpio_write(dev->gpio_dir0, direction);
                /* No brake */
                gpio_write(dev->gpio_dir1, 0 ^ dev->gpio_brake_invert);
                break;

            case MOTOR_BRAKE:
            default:
                /* No brake */
                gpio_write(dev->gpio_dir1, 1 ^ dev->gpio_brake_invert);
                pwm_duty_cycle = 0;
                break;
        }
    }
    else
    {
        /* Stop motor */
        pwm_duty_cycle = 0;
    }

    pwm_set(motor_driver_config[motor_driver].pwm_dev, dev->pwm_channel, pwm_duty_cycle);

#ifdef QDEC_NUMOF
    simulate_encoders(motor_id, direction, pwm_duty_cycle);
#endif /* QDEC_NUMOF */

    return 0;
}

void motor_enable(const motor_driver_t motor_driver, uint8_t motor_id)
{
    const motor_t *dev = &motor_driver_config[motor_driver].motors[motor_id];

    if (dev->gpio_enable_invert)
    {
        gpio_write(dev->gpio_enable, 1);
    }
    else
    {
        gpio_write(dev->gpio_enable, 0);
    }
}

void motor_disable(const motor_driver_t motor_driver, uint8_t motor_id)
{
    const motor_t *dev = &motor_driver_config[motor_driver].motors[motor_id];

    if (dev->gpio_enable_invert)
    {
        gpio_write(dev->gpio_enable, 0);
    }
    else
    {
        gpio_write(dev->gpio_enable, 1);
    }
}
