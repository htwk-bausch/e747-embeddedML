/**
  ******************************************************************************
  * @file    sine_model_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-05-02T10:45:14+0200
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

#include "sine_model_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_sine_model_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_sine_model_weights_array_u64[161] = {
  0xbed353603f0c6701U, 0xbd7527f0bdf35e5cU, 0xbe737b70be73ed18U, 0x3ca92579bebd558dU,
  0x3e6ea5d8bec91e1cU, 0x3ef58d60be8d78e2U, 0xbe4de0c03f1aa30aU, 0x3e8f8659beaba681U,
  0x3eaf194dU, 0x0U, 0x0U, 0x3f79ac4d00000000U,
  0xbf266fb200000000U, 0xbc8953aa00000000U, 0xbf6b9b23U, 0xbe8b036800000000U,
  0xbe71d1e73ecff5deU, 0x3e734ed2be8e7686U, 0xbe6a9aacbebd9174U, 0x3eb3c7db3ea190ffU,
  0xbf164a6f3e3cd9aeU, 0x3e8b514f3e04cb82U, 0x3e18a04ebebd80e6U, 0x3e128e50be528b56U,
  0x3ed75521bf01ab39U, 0x3ec3c1713ea4b3dbU, 0x3df39f6c3ec1b669U, 0x3f4e6cef3e8d999dU,
  0x3d7bd80abe1d7a22U, 0xbf2bf3d63ebb5b81U, 0xbe8135d43b05d18cU, 0x3e89ab4e3d10a758U,
  0x3e27878ebae19600U, 0xbdf8d0d03ea50203U, 0xbecda29dbd89abe0U, 0xbeb4c7a0bd8e5604U,
  0xbe72f3dcbc871d80U, 0xbea5b9e13ea7a8b9U, 0x3e12608a3eba633dU, 0xbe3a3fd6becf5a98U,
  0x3ec63539be840a51U, 0x3e90a6913ec26313U, 0xbd83d4a0bddde048U, 0xbedfee203e8bcf2bU,
  0x3e204f3fbe9b4af0U, 0x3ea372ec3ebf6e11U, 0xbd0c93403ec8f3aeU, 0xba0d32d43d476290U,
  0xbbce6dc0bc1aed4fU, 0x3dad1464be82330bU, 0xbd9d6cd0bea07690U, 0x3e812d0b3ccad1b0U,
  0xbf8f61f83e9e4a23U, 0x3e8d8f92bebf6675U, 0x3dec8a64be5b02bfU, 0xbe703d2c3e06a53eU,
  0xbed8815b3e960a31U, 0xbec9d675be315190U, 0x3e5985cebed632baU, 0xbe81687a3ea523f3U,
  0x3e99ad473e344686U, 0xbe7e52623e9ce607U, 0x3ec69e73becd1844U, 0xbe84cd84be391f52U,
  0xbe3206e23e665b23U, 0x3eac0eefbeab3245U, 0x3ebfbcb13eb675d3U, 0xbf1434f13eb2ea15U,
  0x3e314add3e3794a2U, 0x3dabb01c3ed8429fU, 0x3e7cf6763e80423bU, 0x3e09cac8be32dc16U,
  0xbea4a7453efecc30U, 0x3cb50a70bd966728U, 0x3eb4dfebbe6aaf75U, 0x3e3aba3b3b5c3d80U,
  0x3e71270ebd266708U, 0xbc9bb8c8be88c4e1U, 0x3ec2e281beb7101dU, 0xbd345c953eae7afdU,
  0xbc94d230beaecfecU, 0xbdafe0d4bebe22b3U, 0x3e9a26613db2dac4U, 0xbd9749a1becfee47U,
  0x3e7981ef3defaadcU, 0x3ea89d363e65528aU, 0x3eb7e7833ee7d2c9U, 0x3db0f69a3d83aad8U,
  0x3d7fc7a0be7d3e8dU, 0xbd224f783c8b26a0U, 0x3e04ddca3d1bb0e0U, 0xbe8121983e929fe9U,
  0xbe25c32abe6ef484U, 0xbe4b47a83de440acU, 0xbd66fcf8bed95e26U, 0x3ea20b6bbde450ecU,
  0x3e9e06e33ec031aeU, 0xbe8b22a6bec8b446U, 0x3d208ce83dfa6854U, 0x3d8d5746be869a76U,
  0x3e109aedbec55c9cU, 0x3c61a868bebf58d8U, 0xbe9e2c0fbcc03ce9U, 0xbd5f38d03ce56bb0U,
  0xbe854b223e977933U, 0x3ec717e1be2e84bcU, 0xbe183546be037d46U, 0x3f0759a63d516730U,
  0xbf63750e3ed75481U, 0x3e76b9123e0d79a6U, 0x3ec93077bf246f0aU, 0xbf4b8e3f3de7af1cU,
  0x3eba5203bdcb1ee0U, 0x3dfea9f4be81b15cU, 0x3edb4797bde8b164U, 0xbd020f703e5d6c26U,
  0x3dc3b46c3e931069U, 0xbdf9f91cbe92ad90U, 0x3e07cf2a3e23f942U, 0xbe8ecd70bdd99c48U,
  0xbdc9175cbe4f8a10U, 0x3d1f7c703ea3c47dU, 0x3e9130093e8fb935U, 0x3db1f519bd978110U,
  0xbea437eebecb5425U, 0xbe09431abe929efaU, 0xbde6983cbe635141U, 0xbe5967113e4f51b6U,
  0x3e296d363e96280bU, 0x3d3474f03ec3beb1U, 0x3e38effabd83ccc8U, 0x3e6b3d1bbc442040U,
  0x3b9176d1be12522cU, 0xbe8259093e168356U, 0xbe2e9ebe3dbe9057U, 0x3eb1e6663b23c480U,
  0x3e199986bec37754U, 0x3e3d36e23e8c2c55U, 0x3ecbb8513e1f6de2U, 0xbe93785dbec12d9cU,
  0x3e6cba9bbeb80004U, 0x3e84dba53eb22e93U, 0xbba660c03efbb939U, 0x3ed82f24bcb999a0U,
  0x3ee918723d4806b8U, 0xbef2994300000000U, 0x3eca9bbaU, 0x3de2f9b3be9ce3f2U,
  0xbed270aeU, 0x3eb98cc43dbed0feU, 0xbd45213100000000U, 0xbe8bb77e3e5c9c34U,
  0xbf827810bf222c48U, 0x3f8b4ae6be50de04U, 0x3e9176d43f8a684bU, 0xbe1612383f3e93f2U,
  0xbeee75243eef34b4U, 0x3fb3d70bbd810f47U, 0x3e4cf409bdb49700U, 0x3eb8273bbe981fcdU,
  0xbeae1f69U,
};


ai_handle g_sine_model_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_sine_model_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

