/******************************************************************************

 @file aes_alt.c

 @brief AES implientation for TI chip

 Group: CMCU, LPRF
 Target Device: CC2652

 ******************************************************************************
 
 Copyright (c) 2017-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc26x2_sdk_2_10_00_44
 Release Date: 2018-04-09 12:59:57
 *****************************************************************************/

#include <openthread/config.h>

#include "mbedtls/aes.h"
#include "aes_alt.h"

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#if defined(MBEDTLS_AES_ALT)

#include <string.h>
#include <utils/code_utils.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/crypto.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)

#define CC26X2R1_AES_KEY_UNUSED (-1)
#define CC26X2R1_AES_CTX_MAGIC (0x7E)

/**
 * bitmap of which key stores are currently used
 */
static uint8_t used_keys = 0;

/**
 * number of active contexts, used for power on/off of the crypto core
 */
static unsigned int ref_num = 0;

void mbedtls_aes_init(mbedtls_aes_context *ctx)
{
    if (ref_num++ == 0)
    {
        Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);
    }

    ctx->magic = CC26X2R1_AES_CTX_MAGIC;
    ctx->key_idx = CC26X2R1_AES_KEY_UNUSED;
}

void mbedtls_aes_free(mbedtls_aes_context *ctx)
{
    if (ctx->magic != CC26X2R1_AES_CTX_MAGIC)
    {
        return;
    }

    if (ctx->key_idx != CC26X2R1_AES_KEY_UNUSED)
    {
        used_keys &= ~(1 << ctx->key_idx);
    }

    if (--ref_num == 0)
    {
        Power_releaseDependency(PowerCC26XX_PERIPH_CRYPTO);
    }

    memset((void *)ctx, 0x00, sizeof(ctx));
}

int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits)
{
    unsigned char key_idx;
    int retval = 0;

    if (ctx->magic != CC26X2R1_AES_CTX_MAGIC)
    {
        return -1;
    }

    if (ctx->key_idx != CC26X2R1_AES_KEY_UNUSED)
    {
        used_keys &= ~(1 << ctx->key_idx);
    }

    /* our hardware only supports 128 bit keys */
    otEXPECT_ACTION(keybits == 128u, retval = MBEDTLS_ERR_AES_INVALID_KEY_LENGTH);

    for (key_idx = 0; ((used_keys >> key_idx) & 0x01) != 0 && key_idx < 8; key_idx++)
    {
        ;
    }

    /* we have no more room for this key */
    otEXPECT_ACTION(key_idx < 8, retval = -2);

    otEXPECT_ACTION(CRYPTOAesLoadKey((uint32_t *)key, key_idx) == AES_SUCCESS, retval = MBEDTLS_ERR_AES_INVALID_KEY_LENGTH);

    used_keys |= (1 << key_idx);
    ctx->key_idx = key_idx;
exit:
    return retval;
}

int mbedtls_aes_setkey_dec(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits)
{
    unsigned char key_idx;
    int retval = 0;

    if (ctx->magic != CC26X2R1_AES_CTX_MAGIC)
    {
        return -1;
    }

    if (ctx->key_idx != CC26X2R1_AES_KEY_UNUSED)
    {
        used_keys &= ~(1 << ctx->key_idx);
    }

    /* our hardware only supports 128 bit keys */
    otEXPECT_ACTION(keybits == 128u, retval = MBEDTLS_ERR_AES_INVALID_KEY_LENGTH);

    for (key_idx = 0; ((used_keys >> key_idx) & 0x01) != 0 && key_idx < 8; key_idx++)
    {
        ;
    }

    /* we have no more room for this key */
    otEXPECT_ACTION(key_idx < 8, retval = -2);

    otEXPECT_ACTION(CRYPTOAesLoadKey((uint32_t *)key, key_idx) == AES_SUCCESS, retval = MBEDTLS_ERR_AES_INVALID_KEY_LENGTH);

    used_keys |= (1 << key_idx);
    ctx->key_idx = key_idx;
exit:
    return retval;
}

/**
 * \brief          AES-ECB block encryption/decryption
 *
 * \param ctx      AES context
 * \param mode     MBEDTLS_AES_ENCRYPT or MBEDTLS_AES_DECRYPT
 * \param input    16-byte input block
 * \param output   16-byte output block
 *
 * \return         0 if successful
 */
int mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx, int mode, const unsigned char input[16], unsigned char output[16])
{
    int retval = -1;

    retval = CRYPTOAesEcb((uint32_t *)input, (uint32_t *)output, ctx->key_idx, mode == MBEDTLS_AES_ENCRYPT, false);
    otEXPECT(retval == AES_SUCCESS);

    while ((retval = CRYPTOAesEcbStatus()) ==  AES_DMA_BSY)
    {
        ;
    }

    CRYPTOAesEcbFinish();

exit:
    return retval;
}

#endif /* MBEDTLS_AES_ALT */

