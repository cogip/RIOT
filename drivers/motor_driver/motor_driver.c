#include "board.h"
#include "motor_driver.h"
#include <errno.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define MODE        PWM_LEFT

int motor_driver_init(motor_driver_t motor_driver)
{
    if (motor_driver >= MOTOR_DRIVER_NUMOF)
    {
        errno = ENODEV;
        goto motor_init_err;
    }

    const motor_driver_config_t *motor_driver_conf = &motor_driver_config[motor_driver];

    pwm_t pwm_dev = motor_driver_conf->pwm_dev;
    uint32_t freq = motor_driver_conf->pwm_frequency;
    uint16_t resol = motor_driver_conf->pwm_resolution;
    /* TODO: Update PWM API to return 0 on success */
    if (pwm_init(pwm_dev, MODE, freq, resol) == 0)
    {
        errno = EINVAL;
        goto motor_init_err;
    }

    for (uint8_t i = 0; i < motor_driver_conf->nb_motors; i++)
    {
        if ((motor_driver_conf->motors[i].gpio_dir0 != GPIO_UNDEF) && \
            (gpio_init(motor_driver_conf->motors[i].gpio_dir0, GPIO_OUT)))
            goto motor_init_err;
        if ((motor_driver_conf->motors[i].gpio_dir1 != GPIO_UNDEF) && \
            (gpio_init(motor_driver_conf->motors[i].gpio_dir1, GPIO_OUT)))
            goto motor_init_err;
        if ((motor_driver_conf->motors[i].gpio_enable != GPIO_UNDEF) && \
            (gpio_init(motor_driver_conf->motors[i].gpio_enable, GPIO_OUT)))
            goto motor_init_err;
    }

    return 0;

motor_init_err:
    return -1;
}

int motor_set(const motor_driver_t motor_driver, uint8_t motor_id, \
    motor_direction_t direction, uint16_t pwm_duty_cycle)
{
    if (motor_driver >= MOTOR_DRIVER_NUMOF)
    {
        errno = ENODEV;
        goto motor_set_err;
    }

    const motor_driver_config_t motor_driver_conf = motor_driver_config[motor_driver];
    const motor_t *dev = &motor_driver_conf.motors[motor_id];

    direction = direction ^ dev->gpio_dir_reverse;

    /* Two direction GPIO, handling brake */
    if (motor_driver_conf.mode == MOTOR_DRIVER_2_DIRS)
    {
        if ((dev->gpio_dir0 == GPIO_UNDEF) || \
            (dev->gpio_dir1 == GPIO_UNDEF))
        {
            errno = ENODEV;
            goto motor_set_err;
        }
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
        if (dev->gpio_dir0 == GPIO_UNDEF)
        {
            errno = ENODEV;
            goto motor_set_err;
        }
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
        if ((dev->gpio_dir0 == GPIO_UNDEF) || \
            (dev->gpio_dir1 == GPIO_UNDEF))
        {
            errno = ENODEV;
            goto motor_set_err;
        }
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
        errno = EINVAL;
        goto motor_set_err;
    }

    pwm_set(motor_driver_config[motor_driver].pwm_dev, dev->pwm_channel, pwm_duty_cycle);

    return 0;

motor_set_err:
    return -1;
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
