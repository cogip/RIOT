#include <errno.h>

/* RIOT includes */
#include <assert.h>
#include <board.h>
#include <log.h>
#include <motor_driver.h>

#define ENABLE_DEBUG    (0)
#include <debug.h>

#ifdef MOTOR_DRIVER_NUMOF

#define MOTOR_DRIVER_MODE        PWM_LEFT

int motor_driver_init(motor_driver_t motor_driver)
{
    assert(motor_driver < MOTOR_DRIVER_NUMOF);

    const motor_driver_config_t *motor_driver_conf = &motor_driver_config[motor_driver];

    pwm_t pwm_dev = motor_driver_conf->pwm_dev;
    uint32_t freq = motor_driver_conf->pwm_frequency;
    uint16_t resol = motor_driver_conf->pwm_resolution;

    uint32_t ret_pwm = pwm_init(pwm_dev, MOTOR_DRIVER_MODE, freq, resol);
    if (ret_pwm != freq)
    {
        errno = EINVAL;
        LOG_ERROR("pwm_init failed\n");
        goto motor_init_err;
    }

    for (uint8_t i = 0; i < motor_driver_conf->nb_motors; i++)
    {
        if ((motor_driver_conf->motors[i].gpio_dir0 != GPIO_UNDEF)
          && (gpio_init(motor_driver_conf->motors[i].gpio_dir0,
          GPIO_OUT))) {
            errno = EIO;
            LOG_ERROR("gpio_dir0 init failed\n");
            goto motor_init_err;
        }
        if ((motor_driver_conf->motors[i].gpio_dir1_or_brake != GPIO_UNDEF)
          && (gpio_init(motor_driver_conf->motors[i].gpio_dir1_or_brake,
          GPIO_OUT))) {
            errno = EIO;
            LOG_ERROR("gpio_dir1_or_brake init failed\n");
            goto motor_init_err;
        }
        if (motor_driver_conf->motors[i].gpio_enable != GPIO_UNDEF) {
            if (gpio_init(motor_driver_conf->motors[i].gpio_enable,
                GPIO_OUT)) {
                errno = EIO;
                LOG_ERROR("gpio_enable init failed\n");
                goto motor_init_err;
            }
            motor_enable(motor_driver, i);
        }
    }

    return 0;

motor_init_err:
    return -errno;
}

int motor_set(const motor_driver_t motor_driver, uint8_t motor_id, \
    motor_direction_t direction, uint16_t pwm_duty_cycle)
{
    assert(motor_driver < MOTOR_DRIVER_NUMOF);

    const motor_driver_config_t *motor_driver_conf = 
        &motor_driver_config[motor_driver];

    assert(motor_id < motor_driver_conf->nb_motors);

    const motor_t *dev = &motor_driver_conf->motors[motor_id];

    direction = direction ^ dev->gpio_dir_reverse;

    /* Two direction GPIO, handling brake */
    if (motor_driver_conf->mode == MOTOR_DRIVER_2_DIRS)
    {
        if ((dev->gpio_dir0 == GPIO_UNDEF) || \
            (dev->gpio_dir1_or_brake == GPIO_UNDEF))
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
                gpio_write(dev->gpio_dir1_or_brake, direction ^ 0x1);
                break;
            case MOTOR_BRAKE:
            default:
                /* Brake */
                gpio_write(dev->gpio_dir0,
                    motor_driver_conf->mode_brake);
                gpio_write(dev->gpio_dir1_or_brake,
                    motor_driver_conf->mode_brake);
                pwm_duty_cycle = 0;
                break;
        }
    }
    /* Single direction GPIO */
    else if (motor_driver_conf->mode == MOTOR_DRIVER_1_DIR)
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
    else if (motor_driver_conf->mode == MOTOR_DRIVER_1_DIR_BRAKE)
    {
        if ((dev->gpio_dir0 == GPIO_UNDEF) || \
            (dev->gpio_dir1_or_brake == GPIO_UNDEF))
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
                gpio_write(dev->gpio_dir1_or_brake, dev->gpio_brake_invert);
                break;

            case MOTOR_BRAKE:
            default:
                /* No brake */
                gpio_write(dev->gpio_dir1_or_brake, 1 ^ dev->gpio_brake_invert);
                pwm_duty_cycle = 0;
                break;
        }
    }
    else
    {
        errno = EINVAL;
        goto motor_set_err;
    }

    pwm_set(motor_driver_conf->pwm_dev, dev->pwm_channel, pwm_duty_cycle);

    motor_driver_cb_t cb = motor_driver_conf->cb;
    if (cb) {
        cb(motor_driver, motor_id, direction, pwm_duty_cycle);
    }

    return 0;

motor_set_err:
    return -errno;
}

void motor_enable(const motor_driver_t motor_driver, uint8_t motor_id)
{
    assert(motor_driver < MOTOR_DRIVER_NUMOF);

    const motor_driver_config_t *motor_driver_conf = 
        &motor_driver_config[motor_driver];

    assert(motor_id < motor_driver_conf->nb_motors);

    const motor_t *dev = &motor_driver_conf->motors[motor_id];

    assert(dev->gpio_enable != GPIO_UNDEF);

    gpio_write(dev->gpio_enable, 1 ^ dev->gpio_enable_invert);
}

void motor_disable(const motor_driver_t motor_driver, uint8_t motor_id)
{
    assert(motor_driver < MOTOR_DRIVER_NUMOF);

    const motor_driver_config_t *motor_driver_conf = 
        &motor_driver_config[motor_driver];

    assert(motor_id < motor_driver_conf->nb_motors);

    const motor_t *dev = &motor_driver_conf->motors[motor_id];

    assert(dev->gpio_enable != GPIO_UNDEF);

    gpio_write(dev->gpio_enable, dev->gpio_enable_invert);
}

#endif /* MOTOR_DRIVER_NUMOF */
