/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr.h>
#include <dt-bindings/zmk/mouse.h>

struct k_work_q *zmk_trackpoint_work_q();
int zmk_trackpoint_init();