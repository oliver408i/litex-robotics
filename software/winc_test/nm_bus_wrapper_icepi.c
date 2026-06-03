/* ATWINC1500 SPI bus wrapper for the IcePi Zero shared aux bus.
 *
 * Maps the WINC host driver's bus calls onto the AuxSPIMaster HAL (aux_spi.*).
 * Each nm_spi_rw() is one CS-framed transaction: assert WINC CS, clock the
 * bytes full-duplex, deassert. Mirrors the reference SAM bus wrappers. */
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "bsp/include/nm_bsp.h"

#include "aux_spi.h"

/* Per-transaction chunk cap for HIF block transfers (nmspi splits anything
 * larger). 1024 amortizes the CS-frame overhead on bulk RX; raise further if
 * throughput tests show chunking still matters. */
#define NM_BUS_MAX_TRX_SZ   1024

tstrNmBusCapabilities egstrNmBusCapabilities = {
    NM_BUS_MAX_TRX_SZ
};

sint8 nm_spi_rw(uint8 *pu8Mosi, uint8 *pu8Miso, uint16 u16Sz)
{
    uint16 i;

    aux_spi_select(&AUX_WINC);              /* WINC clock + CS, held for the frame */
    for (i = 0; i < u16Sz; i++) {
        uint8 tx = (pu8Mosi != NULL) ? pu8Mosi[i] : 0;
        uint8 rx = aux_spi_xfer8(tx);
        if (pu8Miso != NULL)
            pu8Miso[i] = rx;
    }
    aux_spi_deselect();
    return M2M_SUCCESS;
}

sint8 nm_bus_init(void *pvinit)
{
    (void)pvinit;
    /* Park the bus (all CS high), then hard-reset the WINC -- matches the
     * reference ports, which call nm_bsp_reset() at the end of nm_bus_init(). */
    aux_spi_deselect();
    nm_bsp_reset();
    nm_bsp_sleep(1);
    return M2M_SUCCESS;
}

sint8 nm_bus_ioctl(uint8 u8Cmd, void *pvParameter)
{
    sint8 s8Ret = 0;
    switch (u8Cmd) {
        case NM_BUS_IOCTL_RW: {
            tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
            s8Ret = nm_spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
        } break;
        default:
            s8Ret = -1;
            M2M_ERR("invalid ioctl cmd\n");
            break;
    }
    return s8Ret;
}

sint8 nm_bus_deinit(void)
{
    aux_spi_deselect();
    nm_bsp_deinit();
    return M2M_SUCCESS;
}

sint8 nm_bus_reinit(void *config)
{
    (void)config;
    return M2M_SUCCESS;
}
