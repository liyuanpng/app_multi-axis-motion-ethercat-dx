#include <3xC21+dc100.inc>
#include <bldc_motor_config.h>
#include <print.h>
#include <stdio.h>
/* Motor Control Includes */
#include <hall_client.h>
#include <hall_server.h>
#include <qei_server.h>
#include <qei_client.h>
#include <pwm_service_inv.h>
#include <commutation_server.h>
#include <refclk.h>
#include <velocity_ctrl_client.h>
#include <velocity_ctrl_server.h>
#include <profile.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <profile_control.h>
#include <mc_constants.h>
on stdcore[NODE_0_IFM_TILE]:clock clk_adc_0 = XS1_CLKBLK_1;
on stdcore[NODE_0_IFM_TILE]:clock clk_pwm_0 = XS1_CLKBLK_REF;
on stdcore[NODE_1_IFM_TILE]:clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[NODE_1_IFM_TILE]:clock clk_pwm_1 = XS1_CLKBLK_REF;
on stdcore[NODE_2_IFM_TILE]:clock clk_adc_2 = XS1_CLKBLK_1;
on stdcore[NODE_2_IFM_TILE]:clock clk_pwm_2 = XS1_CLKBLK_REF;

/**
 * Multi-axis Hall sensor test function
 *
 * @param c_hall Channel to Hall interface server
 * @param axis_number in a multinode-axis system
 */
void hall_position(chanend c_hall, int axis_number) {
    int position = 0;
    int velocity = 0;
    int direction;

    while (1) {
        /* get position from Hall Sensor */
        {   position, direction}= get_hall_position_absolute(c_hall);

        /* get velocity from Hall Sensor */
        velocity = get_hall_velocity(c_hall);

        switch (axis_number) {
case        0:
        printf("Axis%d: %d\n", axis_number, position);
        break;
        case 1:
        printf("          Axis%d: %d\n", axis_number, position);
        break;
        case 2:
        printf("                    Axis%d: %d\n", axis_number, position);
        break;
        default:
        break;
    }
}
}

/**
 * Test Profile Velocity function
 *
 * 4000                  ________________
 *                      /                \
 *                     /                  \
 *                    /                    \
 *       ____________/                      \
 * 0     <----3s----><1s><------4s------><1s>
 *
 * @param c_velocity_ctrl Channel to velocity control server
 */
void profile_velocity_test(chanend c_velocity_ctrl) {
    int acceleration = 4000; // rpm/s
    int deceleration = 4000; // rpm/s

    int pause_duration = 3; //[s]

    while(pause_duration--)
    {
        printintln(pause_duration + 1);
        delay_seconds(1);
    }


    printstr("Start!");

    set_profile_velocity(4000, acceleration, deceleration,
            MAX_PROFILE_VELOCITY, c_velocity_ctrl);

    delay_seconds(4);

    set_profile_velocity(0, acceleration, deceleration, MAX_PROFILE_VELOCITY,
            c_velocity_ctrl);
}

int main(void) {

    chan c_qei_p1_0, c_qei_p2_0; // node 2 qei channels
    chan c_hall_p1_0, c_hall_p2_0, c_hall_p3_0, c_hall_p4_0, c_hall_p5_0,
            c_hall_p6_0; // node 2 hall channels
    chan c_commutation_p1_0, c_commutation_p2_0, c_commutation_p3_0, c_signal_0; // node 2 commutation channels
    chan c_pwm_ctrl_0, c_adctrig_0; // node 2 pwm channels
    chan c_velocity_ctrl_0; // node 2 velocity control channel
    chan c_watchdog_0; // node 2 WDT channel

    chan c_qei_p1_1, c_qei_p2_1; // node 1 qei channels
    chan c_hall_p1_1, c_hall_p2_1, c_hall_p3_1, c_hall_p4_1, c_hall_p5_1,
            c_hall_p6_1; // node 1 hall channels
    chan c_commutation_p1_1, c_commutation_p2_1, c_commutation_p3_1, c_signal_1; // node 1 commutation channels
    chan c_pwm_ctrl_1, c_adctrig_1; // node 1 pwm channels
    chan c_velocity_ctrl_1; // node 1 velocity control channel
    chan c_watchdog_1; // node 1 WDT channel

    chan c_qei_p1_2, c_qei_p2_2; // node 2 qei channels
    chan c_hall_p1_2, c_hall_p2_2, c_hall_p3_2, c_hall_p4_2, c_hall_p5_2,
            c_hall_p6_2; // node 2 hall channels
    chan c_commutation_p1_2, c_commutation_p2_2, c_commutation_p3_2, c_signal_2; // node 2 commutation channels
    chan c_pwm_ctrl_2, c_adctrig_2; // node 2 pwm channels
    chan c_velocity_ctrl_2; // node 2 velocity control channel
    chan c_watchdog_2; // node 2 WDT channel

    par
    {
        /* C21 node 0 */
        on tile[NODE_0_APP_TILE]:
        {
            par {
                {
                    delay_seconds(2);
                    profile_velocity_test(c_velocity_ctrl_0);
                }
                /* Velocity Control Loop */
                {
                    ctrl_par velocity_ctrl_params_0;
                    filter_par sensor_filter_params_0;
                    hall_par hall_params_0;
                    qei_par qei_params_0;

                    init_velocity_control_param(velocity_ctrl_params_0);

                    init_hall_param(hall_params_0);
                    init_qei_param(qei_params_0);

                    /* Initialize sensor filter length */
                    init_sensor_filter_param(sensor_filter_params_0);

                    /* Control Loop */
                    velocity_control(velocity_ctrl_params_0,
                            sensor_filter_params_0, hall_params_0,
                            qei_params_0, SENSOR_USED, c_hall_p2_0, c_qei_p2_0,
                            c_velocity_ctrl_0, c_commutation_p2_0);
                }
            }
        }

        on tile[NODE_0_IFM_TILE]:
        {
            par
            {
                /* PWM Loop */
                do_pwm_inv_triggered(c_pwm_ctrl_0, c_adctrig_0,
                        p_ifm_dummy_port_0, p_ifm_motor_hi_0, p_ifm_motor_lo_0,
                        clk_pwm_0);

                /* Motor Commutation loop */
                {
                    hall_par hall_params_0;
                    qei_par qei_params_0;
                    commutation_par commutation_params_0;
                    int init_state;
                    init_hall_param(hall_params_0);
                    init_qei_param(qei_params_0);
                    commutation_sinusoidal(c_hall_p1_0, c_qei_p1_0, c_signal_0,
                            c_watchdog_0, c_commutation_p1_0,
                            c_commutation_p2_0, c_commutation_p3_0,
                            c_pwm_ctrl_0, p_ifm_esf_rstn_pwml_pwmh_0,
                            p_ifm_coastn_0, p_ifm_ff1_0, p_ifm_ff2_0,
                            hall_params_0, qei_params_0, commutation_params_0);
                }

                /* Watchdog Server */
                run_watchdog(c_watchdog_0, p_ifm_wd_tick_0,
                        p_ifm_shared_leds_wden_0);

                /* Hall Server */
                {
                    hall_par hall_params_0;
                    run_hall(c_hall_p1_0, c_hall_p2_0, c_hall_p3_0,
                            c_hall_p4_0, c_hall_p5_0, c_hall_p6_0,
                            p_ifm_hall_0, hall_params_0);
                }
            }
        }

        /* C21 Node 1 */
        on tile[NODE_1_APP_TILE]:
        {
            par {
                {
                    delay_seconds(2);
                    profile_velocity_test(c_velocity_ctrl_1);
                }
                /* Velocity Control Loop */
                {
                    ctrl_par velocity_ctrl_params_1;
                    filter_par sensor_filter_params_1;
                    hall_par hall_params_1;
                    qei_par qei_params_1;

                    init_velocity_control_param(velocity_ctrl_params_1);

                    init_hall_param(hall_params_1);
                    init_qei_param(qei_params_1);

                    /* Initialize sensor filter length */
                    init_sensor_filter_param(sensor_filter_params_1);

                    /* Control Loop */
                    velocity_control(velocity_ctrl_params_1,
                            sensor_filter_params_1, hall_params_1,
                            qei_params_1, SENSOR_USED, c_hall_p2_1, c_qei_p2_1,
                            c_velocity_ctrl_1, c_commutation_p2_1);
                }
            }
        }

        on tile[NODE_1_IFM_TILE]:
        {
            par
            {
                /* PWM Loop */
                do_pwm_inv_triggered(c_pwm_ctrl_1, c_adctrig_1,
                        p_ifm_dummy_port_1, p_ifm_motor_hi_1, p_ifm_motor_lo_1,
                        clk_pwm_1);

                /* Motor Commutation loop */
                {
                    hall_par hall_params_1;
                    qei_par qei_params_1;
                    commutation_par commutation_params_1;
                    int init_state;
                    init_hall_param(hall_params_1);
                    init_qei_param(qei_params_1);
                    commutation_sinusoidal(c_hall_p1_1, c_qei_p1_1, c_signal_1,
                            c_watchdog_1, c_commutation_p1_1,
                            c_commutation_p2_1, c_commutation_p3_1,
                            c_pwm_ctrl_1, p_ifm_esf_rstn_pwml_pwmh_1,
                            p_ifm_coastn_1, p_ifm_ff1_1, p_ifm_ff2_1,
                            hall_params_1, qei_params_1, commutation_params_1);
                }

                /* Watchdog Server */
                run_watchdog(c_watchdog_1, p_ifm_wd_tick_1,
                        p_ifm_shared_leds_wden_1);

                /* Hall Server */
                {
                    hall_par hall_params_1;
                    run_hall(c_hall_p1_1, c_hall_p2_1, c_hall_p3_1,
                            c_hall_p4_1, c_hall_p5_1, c_hall_p6_1,
                            p_ifm_hall_1, hall_params_1);
                }
            }
        }

        /* C21 Node 2 */
        on tile[NODE_2_APP_TILE]:
        {
            par {
                {
                    delay_seconds(2);
                    profile_velocity_test(c_velocity_ctrl_2);
                }
                /* Velocity Control Loop */
                {
                    ctrl_par velocity_ctrl_params_2;
                    filter_par sensor_filter_params_2;
                    hall_par hall_params_2;
                    qei_par qei_params_2;

                    init_velocity_control_param(velocity_ctrl_params_2);

                    init_hall_param(hall_params_2);
                    init_qei_param(qei_params_2);

                    /* Initialize sensor filter length */
                    init_sensor_filter_param(sensor_filter_params_2);

                    /* Control Loop */
                    velocity_control(velocity_ctrl_params_2,
                            sensor_filter_params_2, hall_params_2,
                            qei_params_2, SENSOR_USED, c_hall_p2_2, c_qei_p2_2,
                            c_velocity_ctrl_2, c_commutation_p2_2);
                }
            }
        }

        on tile[NODE_2_IFM_TILE]:
        {
            par
            {
                /* PWM Loop */
                do_pwm_inv_triggered(c_pwm_ctrl_2, c_adctrig_2,
                        p_ifm_dummy_port_2, p_ifm_motor_hi_2, p_ifm_motor_lo_2,
                        clk_pwm_2);

                /* Motor Commutation loop */
                {
                    hall_par hall_params_2;
                    qei_par qei_params_2;
                    commutation_par commutation_params_2;
                    int init_state;
                    init_hall_param(hall_params_2);
                    init_qei_param(qei_params_2);
                    commutation_sinusoidal(c_hall_p1_2, c_qei_p1_2, c_signal_2,
                            c_watchdog_2, c_commutation_p1_2,
                            c_commutation_p2_2, c_commutation_p3_2,
                            c_pwm_ctrl_2, p_ifm_esf_rstn_pwml_pwmh_2,
                            p_ifm_coastn_2, p_ifm_ff1_2, p_ifm_ff2_2,
                            hall_params_2, qei_params_2, commutation_params_2);
                }

                /* Watchdog Server */
                run_watchdog(c_watchdog_2, p_ifm_wd_tick_2,
                        p_ifm_shared_leds_wden_2);

                /* Hall Server */
                {
                    hall_par hall_params_2;
                    run_hall(c_hall_p1_2, c_hall_p2_2, c_hall_p3_2,
                            c_hall_p4_2, c_hall_p5_2, c_hall_p6_2,
                            p_ifm_hall_2, hall_params_2);
                }
            }
        }
    }

    return 0;
}
