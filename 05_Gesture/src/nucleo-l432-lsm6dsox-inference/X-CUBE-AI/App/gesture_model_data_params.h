/**
  ******************************************************************************
  * @file    gesture_model_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-05-15T12:18:37+0200
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef GESTURE_MODEL_DATA_PARAMS_H
#define GESTURE_MODEL_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_GESTURE_MODEL_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_gesture_model_data_weights_params[1]))
*/

#define AI_GESTURE_MODEL_DATA_CONFIG               (NULL)


#define AI_GESTURE_MODEL_DATA_ACTIVATIONS_SIZES \
  { 2696, }
#define AI_GESTURE_MODEL_DATA_ACTIVATIONS_SIZE     (2696)
#define AI_GESTURE_MODEL_DATA_ACTIVATIONS_COUNT    (1)
#define AI_GESTURE_MODEL_DATA_ACTIVATION_1_SIZE    (2696)



#define AI_GESTURE_MODEL_DATA_WEIGHTS_SIZES \
  { 128188, }
#define AI_GESTURE_MODEL_DATA_WEIGHTS_SIZE         (128188)
#define AI_GESTURE_MODEL_DATA_WEIGHTS_COUNT        (1)
#define AI_GESTURE_MODEL_DATA_WEIGHT_1_SIZE        (128188)



#define AI_GESTURE_MODEL_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_gesture_model_activations_table[1])

extern ai_handle g_gesture_model_activations_table[1 + 2];



#define AI_GESTURE_MODEL_DATA_WEIGHTS_TABLE_GET() \
  (&g_gesture_model_weights_table[1])

extern ai_handle g_gesture_model_weights_table[1 + 2];


#endif    /* GESTURE_MODEL_DATA_PARAMS_H */
