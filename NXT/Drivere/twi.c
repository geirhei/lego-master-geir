/*
 * I2C/TWI comminications driver
 * Provide read/write to an I2C/TWI device (in this case the ATMega
 * co-processor). Uses the hardware TWI device in interrupt mode.
 * NOTES
 * This code does not support single byte read/write operation.
 * This code does not support internal register addressing.
 * Runs at high priority interrupt to minimise chance of early
 * write termination (have never seen this but...).
 * For read operations we do not wait for the complete event before 
 * marking the read as over. We do this because the time window for
 * a read when talking to the ATMega is very tight, so finishing
 * slightly early avoids a data over-run. It is a little iffy though!
 */


#include "twi.h"
#include "FreeRTOS.h"
   
// Calculate required clock divisor
#define   I2CClk                        400000L
#define   CLDIV                         (((configCPU_CLOCK_HZ/I2CClk)/2)-3)
// Pins
#define TWCK (1 << 4)
#define TWD (1 << 3)

static enum {
  TWI_UNINITIALISED = 0,
  TWI_FAILED,
  TWI_IDLE,
  TWI_DONE,
  TWI_RX_BUSY,
  TWI_TX_BUSY,
} twi_state;

static unsigned long twi_pending;
static unsigned char *twi_ptr;
static unsigned long twi_mask;

__irq __arm void twi_isr_C(void);

// Accumlate stats
#ifdef USE_STATS
static struct {
  unsigned long rx_done;
  unsigned long tx_done;
  unsigned long bytes_tx;
  unsigned long bytes_rx;
  unsigned long unre;
  unsigned long ovre;
  unsigned long nack;
} twi_stats;
#define STATS(code) code;
#else
#define STATS(code)
#endif

/**
 * Return the status of the twi device.
 * 0 == Ready for use
 * 1 == Busy
 * -1 == Error or closed
 */
int twi_status(void)
{
  return (twi_state > TWI_DONE ? 1 : (twi_state < TWI_IDLE ? -1 : 0));
}

/**
 * Process TWI interrupts.
 * Assumes that only valid interrupts will be enabled and that twi_mask
 * will have been set to only contain the valid bits for the current
 * I/O state. This means that we do not have to test this state at
 * interrupt time.
 */
__irq __arm void twi_isr_C(void)
{
  unsigned long status = *AT91C_TWI_SR & twi_mask;
  if (status & AT91C_TWI_RXRDY) {
    STATS(twi_stats.bytes_rx++)
    *twi_ptr++ = *AT91C_TWI_RHR;
    twi_pending--;
    if (twi_pending == 1) {
      /* second last byte -- issue a stop on the next byte */
      *AT91C_TWI_CR = AT91C_TWI_STOP;
    }
    if (!twi_pending) {
      // All bytes have been sent. Mark operation as complete.
      STATS(twi_stats.rx_done++)
      twi_state = TWI_DONE;
      *AT91C_TWI_IDR = AT91C_TWI_RXRDY;
    }
  }
  else if (status & AT91C_TWI_TXRDY) {
    if (twi_pending) {
      /* Still Stuff to send */
      *AT91C_TWI_THR = *twi_ptr++;
      twi_pending--;
      STATS(twi_stats.bytes_tx++)
    } else {
      // everything has been sent, now wait for complete 
      STATS(twi_stats.tx_done++);
      *AT91C_TWI_IDR = AT91C_TWI_TXRDY;
      *AT91C_TWI_IER = AT91C_TWI_TXCOMP;
      twi_mask = AT91C_TWI_TXCOMP|AT91C_TWI_NACK;
    }
  }
  else if (status & AT91C_TWI_TXCOMP) {
    twi_state = TWI_DONE;
    *AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
  }

  if (status & AT91C_TWI_NACK) {
    STATS(twi_stats.nack++)
    *AT91C_TWI_IDR = ~0;
    twi_state = TWI_UNINITIALISED;
  }
  
  AT91C_BASE_AIC->AIC_EOICR = 0; //Inform the AIC the interrupt is done
}


/**
 * Force a device reset. 
 */
void twi_reset(void)
{
  unsigned long clocks = 9;

  *AT91C_TWI_IDR = ~0;

  *AT91C_PMC_PCER = (1 << AT91C_ID_PIOA) |	/* Need PIO too */
                    (1 << AT91C_ID_TWI);	/* TWI clock domain */

  /* Set up pin as an IO pin for clocking till clean */
  *AT91C_PIOA_MDER = TWD | TWCK;
  *AT91C_PIOA_PER = TWD | TWCK;
  *AT91C_PIOA_ODR = TWD;
  *AT91C_PIOA_OER = TWCK;

  while (clocks > 0 && !(*AT91C_PIOA_PDSR & TWD)) {

    *AT91C_PIOA_CODR = TWCK;
    
    int x = (1500 >> 7) + 1;
    while (x) {
      x--;
    }
    
    *AT91C_PIOA_SODR = TWCK;
    
    x = (1500 >> 7) + 1;
    while (x) {
      x--;
    }
    
    clocks--;
  }

  *AT91C_PIOA_PDR = TWD | TWCK;
  *AT91C_PIOA_ASR = TWD | TWCK;

  *AT91C_TWI_CR = AT91C_TWI_SWRST|AT91C_TWI_MSDIS;/* Disable & reset */

  *AT91C_TWI_CWGR = ((CLDIV << 8)|CLDIV);       /* Set for 400kHz */
  *AT91C_TWI_CR = AT91C_TWI_MSEN;		/* Enable as master */
  *AT91C_TWI_IER = AT91C_TWI_NACK;
  twi_mask = 0;
}

/**
 * Initialize the device.
 */
int twi_init(void)
{
  *AT91C_TWI_IDR = ~0;		/* Disable all interrupt sources */

  AT91F_AIC_ConfigureIt( AT91C_BASE_AIC, AT91C_ID_TWI, AT91C_AIC_PRIOR_HIGHEST-2, AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, ( void (*)(void) ) twi_isr_C );
  AT91F_AIC_EnableIt( AT91C_BASE_AIC, AT91C_ID_TWI );    

  twi_reset();

  /* Init peripheral */

  twi_state = TWI_IDLE;

  return 1;
}


/**
 * Start a read operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_read(unsigned long dev_addr, unsigned char *data, unsigned long nBytes)
{
  if (twi_state < TWI_RX_BUSY) {
    twi_state = TWI_RX_BUSY;
    twi_ptr = data;
    twi_pending = nBytes;
    *AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO|AT91C_TWI_MREAD|((dev_addr & 0x7f) << 16);
    twi_mask = AT91C_TWI_RXRDY|AT91C_TWI_NACK;
    *AT91C_TWI_CR = AT91C_TWI_START;
    *AT91C_TWI_IER = AT91C_TWI_RXRDY;
  }

}

/**
 * Start a write operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_write(unsigned long dev_addr, const unsigned char *data, unsigned long nBytes)
{
  if (twi_state < TWI_RX_BUSY) {
    twi_state = TWI_TX_BUSY;
    twi_ptr = (unsigned char *)data;
    twi_pending = nBytes;

    *AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO|((dev_addr & 0x7f) << 16);
    *AT91C_TWI_THR = *twi_ptr++;
    twi_pending--;
    STATS(twi_stats.bytes_tx++)
    twi_mask = AT91C_TWI_TXRDY|AT91C_TWI_NACK;
    *AT91C_TWI_IER = AT91C_TWI_TXRDY;
  }

}
