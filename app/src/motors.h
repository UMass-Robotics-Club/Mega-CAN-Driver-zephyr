/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * The four Robstride motors, one per CAN bus (motor i on can<i>).
 * One protocol tick = at most one CAN frame per motor; every frame the motor
 * receives is answered with a type 2 feedback frame, collected here.
 */

#ifndef MEGA_CAN_MOTORS_H_
#define MEGA_CAN_MOTORS_H_

#include <stdbool.h>
#include <zephyr/kernel.h>

#include "proto.h"

/* Start the buses and set each motor's CAN_TIMEOUT (motor-side watchdog). */
int motors_init(void);

/*
 * One tick. cmd[i] is motor i's slot from the command packet. While !armed,
 * MODE_RUN is treated as MODE_OFF.
 */
void motors_apply(const struct proto_cmd_motor cmd[PROTO_NUM_MOTORS], bool armed);

/* Stop every motor (watchdog trip). */
void motors_all_off(void);

/* Wait until every motor sent in the last tick has answered, or timeout. */
void motors_wait_feedback(k_timeout_t timeout);

/* Latest motor feedback and CAN bus health, in protocol form. */
void motors_fill_state(struct proto_st_motor motor[PROTO_NUM_MOTORS],
		       struct proto_st_bus bus[PROTO_NUM_BUSES]);

#endif /* MEGA_CAN_MOTORS_H_ */
