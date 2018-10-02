/********************************************************************
 * This library is based on the the original :  
 * 
 * TwoWire.cpp - TWI/I2C library for Arduino & Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
 * Modified December 2014 by Ivan Grokhotkov (ivan@esp8266.com) - esp8266 support
 * Modified April 2015 by Hrsto Gochkov (ficeto@ficeto.com) - alternative esp8266 support
 * 
 * *********************************************************************
 * Modified September 2018 for Raspberry Pi.
 * by Paul van Haastrecht (paulvha@hotmail.com) :
 * 
 * It has been adjusted and extended to work on a Raspberry Pi. 
 * This library will support BOTH hardware and software I2C communication.
 * The software i2C (bit banging) can now be handled on any pin, but 
 * also provides support for clock Stretching for slow I2C devices. 
 * Clock stretching on the BCM2835 Chip is badly (read: NOT working) implemented.
 * 
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * 
 * ********************************************************************
 * twowire is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 *
 * twowire is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with twowire.  If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************
 * 
 * initial version 1.0 for Raspberry Pi
 * *******************************************************************
 * October 2018
 * 
 * added 
 *     setPullup() call to allow adding pull-up resistors to SDA & SCL
 *********************************************************************/

# include <bcm2835.h>
# include <stdio.h>
# include "twowire.h"

///////////////////////////////////////////////////////////////////////
// Initialize Class Variables for soft implementation  ////////////////
///////////////////////////////////////////////////////////////////////

uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;

///////////////////////////////////////////////////////////////////
// define structures //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

twowire_s twSetting;
clock_stretch clck_str_info;

////////////////////////////////////////////////////////////////////
// Constructors ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

TwoWire::TwoWire(){             // i2c interface
    
    /* default hardware I2C */
    twSetting.hw_interface = hard_I2C;
    
    /* initialize status */
    twSetting.hw_initialized = false;
    
    /* indicate NO Slave address */
    twSetting.Slave_address = 0x0;
    
    /* set default speed */
    twSetting.baudrate = default_si2c_clock;
    
    /* default display NO driver messages */
    twSetting.dbmes = false;
    
    /* indicate NO internal pull-up resistors */
    twSetting.Pullup = false;
} 
           
TwCore::TwCore(){}              // software I2C implementation
BcmCore::BcmCore(){}            // BCM2835 hardware I2C implementation

//////////////////////////////////////////////////////////////////////
// Public Methods ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

/********************************
 * set slave address
 *********************************/
 
void TwoWire::setSlave(uint8_t address) {
    twSetting.Slave_address = address; 
}

/*****************************************************************
 * set for pull-up resistors on the SDA / SCL GPIO line
 * 
 * Some slave devices  don't have them and instead of adding external 
 * resistors, use the internal pull-up resistors from the BCM2835
 ******************************************************************/
 
void TwoWire::setPullup() {
    twSetting.Pullup = true;
}
 
/**********************************
 * enable / disable  driver messages
 **********************************/
 
void TwoWire::setDebug(bool db) {
    twSetting.dbmes = db; 
} 

/******************************************
 * begin with initialisation
 * 
 * SDA and SCL must be GPIO numbers and only
 * used for soft_I2C
 * 
 * set I2C interface with itoc
 * hard_I2C = BCM2835 / hardware implemenation
 * soft_I2C = software implementation
 ******************************************/
 
Wstatus TwoWire::begin(bool itoc, int sda, int scl){
  Wstatus ret;
  
  /* if already initialized */
  if (twSetting.hw_initialized) return(TW_SUCCESS);
  
  twSetting.twi_sda  = sda;
  twSetting.twi_scl  = scl;
  twSetting.hw_interface = itoc; 

  /* initialize hardware correctly */
  if (twSetting.hw_interface == hard_I2C) 
    ret = bcm_init();
  
  else if (twSetting.hw_interface == soft_I2C) 
  {
    ret = twi_init();
    
    /* flush any buffers pointers or input */
    flush();
  }
  else
    ret = TW_PARAM_ERROR;
  
  if (ret != TW_SUCCESS) return(ret);

  /* indicate that hardware has been set */
  twSetting.hw_initialized = true;
    
  return(TW_SUCCESS);
}

/******************************************
 * reset BCM2835 and release memory
 ******************************************/
 
void TwoWire::close() {

  /* if not initialized (anymore) */
  if (! twSetting.hw_initialized) return;
  
  /* reset the SDA/SCL line */
  if (twSetting.hw_interface == soft_I2C) 
  {
    twi_stop();
    
    /* if pull-up resistors was set, remove them */
    if (twSetting.Pullup)
    {
        bcm2835_gpio_set_pud(twSetting.twi_sda, BCM2835_GPIO_PUD_OFF);
        bcm2835_gpio_set_pud(twSetting.twi_scl, BCM2835_GPIO_PUD_OFF);
    }
  }
  
  /* release memory */
  bcm_close();
  
  /* indicate that hardware has been reset */
  twSetting.hw_initialized = false;
}  

/******************************************
 * check SCL and SDA line status
 ******************************************/

Wstatus TwoWire::linestatus(){
    return twi_status();
}

/******************************************
 * set speed and timing
 ******************************************/

void TwoWire::setClock(uint16_t frequency){

  /* check on boundery */
  if (frequency < 1 || frequency > 400) return;
   
  twSetting.baudrate = frequency;
    
  /* set BCM2835 */
  if (twSetting.hw_interface == hard_I2C)
    bcm2835_i2c_set_baudrate(frequency * 1000);

  /* set soft_I2C */
  else twi_setClock(frequency);
}

/******************************************
 * set maximum clock stretch
 * only software I2C
 ******************************************/
 
void TwoWire::setClockStretchLimit(uint32_t limit){
    twi_setClockStretchLimit(limit);
}

/*******************************************
 * clear Clock Stretch statistics
 * only software I2C
 *******************************************/
void TwoWire::ClrStretchStat(){
    twi_ClrStretchStat();
}

/*************************************************
 * get Clock Stretch statistics
 * only software I2C
 *************************************************/
void TwoWire::GetStretchStat(clock_stretch *stat){
    
    /* no statistics on Clock stretch with hardware I2C */
    if (twSetting.hw_interface == hard_I2C) {
        stat->clockStretchCount = 1 + clck_str_info.clockStretchCount * ClkStrCor;
        stat->r_clock_stretch_max = 0xff;
        stat->r_clock_stretch_min = 0xff;
        stat->r_clock_stretch_tot = 0xff;
        stat->r_clock_stretch_cnt = 0xff;
        stat->r_clock_stretch_err = 0xff;
        stat->w_clock_stretch_max = 0xff;
        stat->w_clock_stretch_min = 0xff;
        stat->w_clock_stretch_tot = 0xff;
        stat->w_clock_stretch_cnt = 0xff;
        stat->w_clock_stretch_err = 0xff;
        return;
    }
    
    twi_GetStretchStat(stat);
}

/****************************************
 * Display the clock stretch info
 * 
 * only real usage is for diagnostics
 *****************************************/
void TwoWire::DispClockStretch()
{
    struct clock_stretch stat;
    
    GetStretchStat(&stat);
    
    printf("\nclock_stretch_limit\t\t%d uS\n", stat.clockStretchCount);
    printf("\tmax\tmin\ttotal time\tevents\terrors\n");
    printf("\tuS\tuS\tuS\t\t#\t#\n");
    printf("read\t%d\t%d\t%d\t\t%d\t%d\n",
    stat.r_clock_stretch_max, stat.r_clock_stretch_min, stat.r_clock_stretch_tot,
    stat.r_clock_stretch_cnt, stat.r_clock_stretch_err);

    printf("write\t%d\t%d\t%d\t\t%d\t%d\n\n",
    stat.w_clock_stretch_max, stat.w_clock_stretch_min, stat.w_clock_stretch_tot,
    stat.w_clock_stretch_cnt, stat.w_clock_stretch_err);    
    
    ClrStretchStat();
}

/********************************************************
 * write buffer of data to slave 
 * slave address must have been provided before
 ********************************************************/
 
Wstatus TwoWire::i2c_write(char *buff, uint8_t length) {
    
    /* slave address was not set */
    if (twSetting.Slave_address == 0) return(TW_PARAM_ERROR);

    /* if not initialized  */
    if (! twSetting.hw_initialized) return(TW_GENERIC_ERROR);
    
    /* hardware I2C */
    if (twSetting.hw_interface == hard_I2C) return(bcm_write(buff, length));

    /************ soft I2C ***********/
    
    /* set pointers */
    beginTransmission();
    
    /* add data to tx_buffer */
    if(write(buff, length) != length) return(I2C_SDA_DATA);
   
    /* sent data and end transmission */
    return(endTransmission());
}

/****************************************************
 * read data from slave 
 * Slave address must have been provided before
 ****************************************************/

Wstatus TwoWire::i2c_read(char *buff, uint8_t length) {
   
    /* slave address was not set */
    if (twSetting.Slave_address == 0) return(TW_PARAM_ERROR);
    
    /* if not initialized  */
    if (! twSetting.hw_initialized) return(TW_GENERIC_ERROR);
    
    /* hardware I2C */
    if (twSetting.hw_interface == hard_I2C) return(bcm_read(buff, length));
    
    /* soft I2C : read into buffer and stop */
    return(twi_readFrom((uint8_t *) buff, length, 1));
}

/************************************************************************
 * Read from slave.  Slave address must have been provided before
 * 
 * Read a number of bytes from I2C sending a repeated start after writing
 * the required register. Only works if your slave supports this mode
 ************************************************************************/

Wstatus TwoWire::i2c_read_rs(char * regaddr, char *buff, uint8_t length) {

    /* slave address was not set */
    if (twSetting.Slave_address == 0) return(TW_PARAM_ERROR);
    
    /* if not initialized  */
    if (! twSetting.hw_initialized) return(TW_GENERIC_ERROR);
    
    /* hardware I2C */
    if (twSetting.hw_interface == hard_I2C)
        return(bcm_read_rs(regaddr, buff, length));

    /* soft I2C : read into buffer with restart and stop */
    return(twi_readFrom_rs(regaddr,  (uint8_t *) buff, length));
}

/******************************************
 * initialise start transmission
 * software I2C
 ******************************************/

void TwoWire::beginTransmission() {
    
  transmitting = 1;     // indicate transmit sequence started
  txBufferIndex = 0;    // reset pointers
  txBufferLength = 0;
}

/*****************************************************
 * software I2C 
 * start the real sending of the slave address
 * and data. Then end transmission if sendStop was set
 *****************************************************/

Wstatus TwoWire::endTransmission(uint8_t sendStop) {
    
  Wstatus ret = twi_writeTo(txBuffer, txBufferLength, sendStop);
  transmitting = 0;     // indicate transmit ended
  return ret;
}

Wstatus TwoWire::endTransmission(void) {
  return endTransmission(true);
}

/********************************************
 * software I2C
 * write single byte into the txbuffer
 * unless the buffer length has been exceeded
 * 
 * return: byte added
 ********************************************/

size_t TwoWire::write(char data) {
       
    if(transmitting)    // set during beginTransmission
    {
        if(txBufferLength >= BUFFER_LENGTH)  return 0;
        
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        txBufferLength = txBufferIndex;
        
        return 1;
    } 
    
    return 0;
}

/***********************************************
 * software I2C
 * write buffer of data info the tx buffer
 * 
 * return the quantity added
 **********************************************/
size_t TwoWire::write(char *data, uint8_t quantity) {
    
    for(size_t i = 0; i < quantity; ++i)
    {
      if(!write(data[i])) return i;
    }

    return quantity;
}

/***********************************************************
 * software I2C
 * 
 * flush buffers
 ***********************************************************/

void TwoWire::flush(void) {
  
  if (twSetting.hw_interface == hard_I2C)  return;
  
  /* reset pointers */
  txBufferIndex = 0;
  txBufferLength = 0;
  
  /* just in case something was still being sent by slave */
  twi_flush();
}

///////////////////////////////////////////////////////////
// twowire core / Software low level //////////////////////
///////////////////////////////////////////////////////////

/****************************
 *  handle SCL and SDA line 
 ****************************/

#define SDA_LOW()   (SET_L(twSetting.twi_sda,LOW))      
#define SDA_HIGH()  (SET_L(twSetting.twi_sda,HIGH))
#define SDA_READ()  (bcm2835_gpio_lev(twSetting.twi_sda))         
#define SCL_LOW()   (SET_L(twSetting.twi_scl,LOW))
#define SCL_HIGH()  (SET_L(twSetting.twi_scl,HIGH))  
#define SCL_READ()  (bcm2835_gpio_lev(twSetting.twi_scl))

void SET_L (unsigned char line, bool level) {
    
    if (!level)     // set low
    {
        bcm2835_gpio_fsel(line, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(line, LOW);
    }
    else            // enable high
    {
        bcm2835_gpio_fsel(line, BCM2835_GPIO_FSEL_INPT);
    }
}

/*****************************************************
 * software I2C 
 * calculate the clock pulse time for software
 ******************************************************/

void TwCore::twi_setClock(uint16_t freq) {
    
  /* frequency is in Khz : e.g. 100KHz = 10 us
   * we sample in the middle,thus twi_dcount = 5us
   * 
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   * !!! The maximum speed is between 166Khz and 200Khz !!!
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   * 
   * twi_dcount is minimum 1, full cycle is 2 x dcount = 2
   * twi_delay takes 1.8us and as such the min cycle time is 3.6us (277Khz)
   * 
   * HOWEVER due to code overhead the min. cycle time is 5us, sometimes
   * 6us (measured with scope)
   * As such 200Khz - 166Khz is the maximum speed for soft_I2C.
   * 
   */
  twSetting.twi_dcount = (1000 / freq) / 2;
   
  /* fine tuning : subtract 1us because of twi_delay code overhead */
  if ( twSetting.twi_dcount > 1)  twSetting.twi_dcount -= 1;
}

/************************************************
 * software I2C 
 * set the clock stretch maximum corrected limit
 * 
 * correction is needed to fine tune as the code with twi_delay()
 * will add overhead time. 
 ************************************************/
 
void TwCore::twi_setClockStretchLimit(uint32_t limit){
  
  clck_str_info.clockStretchCount = limit / ClkStrCor;
  
  /* clear statistics */
  twi_ClrStretchStat();
}

/********************************************
 * software I2C
 * log the clock stretch statistics
 * count : number of usec before released
 * err : 1 = exceeded limit, 0 = not exceeded limit
 * act : 1 = during reading, 0 = during writing
 ********************************************/

void TwCore::twi_LogClockStretchStat(uint32_t count, int err, bool act) {
    
    if (count < 2) return;
    
    /*! set reading statistics */
    if (act)
    {
        // set max
        if (count > clck_str_info.r_clock_stretch_max)
            clck_str_info.r_clock_stretch_max = count;
        
        // set minimum count    
        if ( clck_str_info.r_clock_stretch_min == 0)
            clck_str_info.r_clock_stretch_min = count;
        
        else if (count < clck_str_info.r_clock_stretch_min)
            clck_str_info.r_clock_stretch_min = count;
        
        // set total count
        clck_str_info.r_clock_stretch_tot += count;
        
        // set number of times clock stretch happend
        // can be used to caculate the average
        clck_str_info.r_clock_stretch_cnt++;
        
        // set error count
        if (err) clck_str_info.r_clock_stretch_err++;
    }
    /*! set writing statistics */
    else
    {
        // set max
        if (count > clck_str_info.w_clock_stretch_max)
            clck_str_info.w_clock_stretch_max = count;
        
        // set minimum count    
        if ( clck_str_info.w_clock_stretch_min == 0)
            clck_str_info.w_clock_stretch_min = count;
        
        else if (count < clck_str_info.w_clock_stretch_min)
            clck_str_info.w_clock_stretch_min = count;
        
        // set total count
        clck_str_info.w_clock_stretch_tot += count;
        
        // set number of times clock stretch happend
        // can be used to caculate the average
        clck_str_info.w_clock_stretch_cnt++;
        
        // set error count
        if (err) clck_str_info.w_clock_stretch_err++;
    }
}

/*******************************************************
 * software I2C
 * get Clock Stretch statistics
 * Apply the correction factor for twi_delay() overhead
 *******************************************************/

void TwCore::twi_GetStretchStat(clock_stretch *stat) {
    
    stat->clockStretchCount = 1+ clck_str_info.clockStretchCount  * ClkStrCor;
    stat->r_clock_stretch_max = clck_str_info.r_clock_stretch_max * ClkStrCor;
    stat->r_clock_stretch_min = clck_str_info.r_clock_stretch_min * ClkStrCor;
    stat->r_clock_stretch_tot = clck_str_info.r_clock_stretch_tot * ClkStrCor;
    stat->r_clock_stretch_cnt = clck_str_info.r_clock_stretch_cnt ;
    stat->r_clock_stretch_err = clck_str_info.r_clock_stretch_err ;
    stat->w_clock_stretch_max = clck_str_info.w_clock_stretch_max * ClkStrCor;
    stat->w_clock_stretch_min = clck_str_info.w_clock_stretch_min * ClkStrCor;
    stat->w_clock_stretch_tot = clck_str_info.w_clock_stretch_tot * ClkStrCor;
    stat->w_clock_stretch_cnt = clck_str_info.w_clock_stretch_cnt ;
    stat->w_clock_stretch_err = clck_str_info.w_clock_stretch_err ;
}

/*************************************************
 * software I2C
 * clear Clock Stretch statistics
 *************************************************/

void TwCore::twi_ClrStretchStat() {
    
    clck_str_info.r_clock_stretch_max = 0;
    clck_str_info.r_clock_stretch_min = 0;
    clck_str_info.r_clock_stretch_cnt = 0;
    clck_str_info.r_clock_stretch_tot = 0;
    clck_str_info.r_clock_stretch_err = 0;
    clck_str_info.w_clock_stretch_max = 0;
    clck_str_info.w_clock_stretch_min = 0;
    clck_str_info.w_clock_stretch_cnt = 0;
    clck_str_info.w_clock_stretch_tot = 0;
    clck_str_info.w_clock_stretch_err = 0;
}


/*************************************************
 * software I2C
 * initialise the hardware and counters
 * for software I2C implementation (bit-banging)
 *************************************************/

Wstatus TwCore::twi_init() {
    
  if (!bcm2835_init()) 
  {
    db_print((char *)"Can't init bcm2835!\n");
    return(TW_INTERNAL_ERROR);
  }
  
  /* set a pull-up resistor was requested. 
   * Not needed in case of GPIO 2 and 3 these are hardwired on the 
   * board already 
   * 
   * Be aware the pull-up resistor are about 10K, compared to 1.8K pm
   * GPIO 2 and 3. In case of long lines, external resistors are 
   * preferred 
   */
   
  if (twSetting.Pullup)
  {
      if (twSetting.twi_sda != 2)
        bcm2835_gpio_set_pud(twSetting.twi_sda, BCM2835_GPIO_PUD_UP);

      if (twSetting.twi_scl != 3)
        bcm2835_gpio_set_pud(twSetting.twi_scl, BCM2835_GPIO_PUD_UP);
  }
  
  /* set sda/scl pins floating */
  twi_stop();

  /* set default clock speed */
  twi_setClock(default_si2c_clock);
  
  /* set default clockstretch */
  twi_setClockStretchLimit(default_si2c_stretch);

  return(TW_SUCCESS);
}

/********************************************
 * software I2C 
 * Stop any output, set floating
 ********************************************/

void TwCore::twi_stop(void){
    
  bcm2835_gpio_fsel(twSetting.twi_sda, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(twSetting.twi_scl, BCM2835_GPIO_FSEL_INPT);
}

/*******************************************************************
 * software I2C
 * 
 * Perform a delay. one loop of 1usec takes ~1.8 usec due to program 
 * overhead of ~0.8us.
 * 
 * we could have done a for-loop and fine tune to exactly 1us, but 
 * a loop takes processor time and this would be high processor load 
 * If running multiple versions in parallel also unpredictable results
 ******************************************************************/

void TwCore::twi_delay(uint64_t usec){
    bcm2835_delayMicroseconds(usec);
}

/**********************************************
 * software I2C
 * 
 * start : SDA from high to low as SCL is HIGH
 * 
 * return : 
 *  FALSE in case of line SDA not HIGH as expected
 *  TRUE if OK
 **********************************************/
 
bool TwCore::twi_write_start(void) {
    
  SCL_HIGH();
  SDA_HIGH();

  /* error in cabling */
  if (SDA_READ() == 0) return(false);
   
  /* wait half clock time */
  twi_delay(twSetting.twi_dcount);

  /* set SDA_LOW to indicate start */
  SDA_LOW();

  /* wait second half */
  twi_delay(twSetting.twi_dcount);
  
  /* finalise with better timing */
  SCL_LOW();
  
  twi_delay(twSetting.twi_dcount);
  
  return(true);
}

/**********************************************
 * software I2C
 * 
 * stop : SDA from low to high as SCL is HIGH
 * 
 * return : 
 *  false: stop failed
 **********************************************/

bool TwCore::twi_write_stop(void){
  
  uint32_t i = 0;
  SCL_LOW();
  SDA_LOW();
  
  /* delay for half clock cycle */
  twi_delay(twSetting.twi_dcount);
  
  SCL_HIGH();
  
  while (SCL_READ() == 0 && (i++) < clck_str_info.clockStretchCount)
    twi_delay(1);
  
  /* if clock stretch expired, indicate error */
  if (i == clck_str_info.clockStretchCount)
  {
      twi_LogClockStretchStat(i, 1, 0);
      twi_stop();
      return (false);
  }
  else
  {
    /* log clock stretch statistics */
    twi_LogClockStretchStat(i, 0, 0);
  }
  
  /* wait half clock cycle */
  twi_delay(twSetting.twi_dcount);
  
  /* set STOP */
  SDA_HIGH();
  
  /* wait another half */
  twi_delay(twSetting.twi_dcount);

  return(true);
}

/*********************************************************************
 * software I2C
 * 
 * Write a bit and check for clock stretching
 * 
 * return
 *  TRUE if OK
 *  FALSE if clock stretching timed out
 * 
 * Clock stretching pauses a transaction by the slave, which is holding 
 * the SCL line LOW 
 * 
 * The transaction cannot continue until the line is released HIGH again. 
 * 
 * Clock stretching is optional and in fact, most slave devices do not 
 * include an SCL driver so they are unable to stretch the clock.
 ********************************************************************/

bool TwCore::twi_write_bit(bool bit) {
    
  uint32_t i = 0;
  SCL_LOW();

  if (bit)  SDA_HIGH();
  else      SDA_LOW();
  
  /* wait until half the clock cycle */
  twi_delay(twSetting.twi_dcount);

  /* trigger high SCL */
  SCL_HIGH();
  
  /* wait for SCL to be high */
  while (SCL_READ() == 0 && (i++) < clck_str_info.clockStretchCount)
    twi_delay(1);  // Clock stretching
    
  /* if clock stretch expired, indicate error */
  if (i == clck_str_info.clockStretchCount)
  {
      twi_LogClockStretchStat(i, 1,0);
      twi_stop();
      return (false);
  }
  else
  {
    /* log clock stretch statistics */
    twi_LogClockStretchStat(i, 0,0);
  }

  /* wait for other half clock cycle */
  twi_delay(twSetting.twi_dcount);

  return(true);
}

/*****************************************************
 * software I2C
 * read SDA line value and check for clock stretching
 * 
 * return
 *  TRUE if error
 *  FALSE if OK
 * 
 * Clock stretching pauses a transaction by the slave, which is holding 
 * the SCL line LOW 
 * 
 * The transaction cannot continue until the line is released HIGH again. 
 * 
 * Clock stretching is optional and in fact, most slave devices do not 
 * include an SCL driver so they are unable to stretch the clock.
 *****************************************************/

bool TwCore::twi_read_bit(void) {
    
  uint32_t i = 0;

  SCL_LOW();
  SDA_HIGH();               // release SDA line

  /* wait in the middle of cycle */
  twi_delay(twSetting.twi_dcount);  

  SCL_HIGH();               // pulse SCL

  /* now check whether clock stretching takes too long */
  while (SCL_READ() == 0 && (i++) < clck_str_info.clockStretchCount)
    twi_delay(1);// Clock stretching

  /* if clock stretch expired, indicate error */
  if (i == clck_str_info.clockStretchCount)
  {
      twi_LogClockStretchStat(i, 1, 1);
      twi_stop();
      return (true);    // NACK
  }
  else
  {
    /* log clock stretch statistics */
    twi_LogClockStretchStat(i, 0, 1);
  }
  
  /* read value */
  bool bit = SDA_READ();

  /* finalise the clock cycle */
  twi_delay(twSetting.twi_dcount);

  return bit;
}

/****************************************************
 * software I2C
 * Write 8 bits and check for clock stretching     *
 ****************************************************/

bool TwCore::twi_write_byte(unsigned char byte) {
    
    unsigned char bit;
    
    for (bit = 0; bit < 8; bit++) {
    
        if (twi_write_bit(byte & 0x80) != true)
        {
            return(true);      // NACK
        }
        
        // set to next bit
        byte <<= 1;
    }
    
    return !twi_read_bit(); //NACK/ACK
}

/****************************************************
 * software I2C                                     *
 * Write 8 bits and check for clock stretching      *
 * perform a restart following reading the NACk     *
 ****************************************************/

bool TwCore::twi_write_byte_rs(unsigned char byte) {
    
    unsigned char bit;
    
    for (bit = 0; bit < 8; bit++) {
    
        if (twi_write_bit(byte & 0x80) != true)
        {
            return(true);      // NACK
        }
        
        // set to next bit
        byte <<= 1;
    }
    
    bool bit1 = twi_read_bit();   //NACK/ACK
    
    /* 
     * timing is critical.
     * twi_read_bit() ends with SCL HIGH and half cycle wait, we now 
     * pull it low and wait half clock-cycle to enable restart
     */ 
    SCL_LOW();
    
    /* wait until half the clock cycle */
    twi_delay(twSetting.twi_dcount);
    
    if (twi_write_start() == false)
        return(true);
    
    return (!bit1);
}

/**********************************************
 * software I2C
 * read a byte from I2C device
 * 
 * nack:
 * false will cause a low (ACK to be written)
 * true will cause a high (NACK to be written)
 * 
 * return : byte read
 **********************************************/

uint8_t TwCore::twi_read_byte(bool nack) {

  uint8_t byte = 0;
  uint8_t bit;

  for (bit = 0; bit < 8; bit++)
    byte = (byte << 1) | twi_read_bit();

  twi_write_bit(nack);

  return byte;
}

/*******************************************************************
 * software I2C
 * just in case the slave is sending something
 * clock for max BUFFER_LENGTH times the rubish away
 *******************************************************************/
 
void TwCore::twi_flush() {
    
    int  i = 0;
    
    /* if low on SDA just clock it away */
    while(SDA_READ() == 0 && (i++) < BUFFER_LENGTH){
        SCL_LOW();
        twi_delay(twSetting.twi_dcount);
        SCL_HIGH();
        twi_delay(twSetting.twi_dcount);
    }
}
 
/************************************************* 
 * software I2C
 * 
 * write data from buffer to address
 * buf      : buffer with data
 * len      : length of data in buffer
 * stop     : TRUE : sent STOP after transmitting
 * 
 *************************************************/
 
Wstatus TwCore::twi_writeTo(uint8_t * buf, uint8_t len, uint8_t sendStop){
 
  uint8_t i;
 
  /* start, pull SDA LOW, while SCL is high */
  if(! twi_write_start())
    return(I2C_SDA_HELD_LOW); //line busy

  /* sent address first */
  if(!twi_write_byte(((twSetting.Slave_address  << 1) | 0) & 0xFF)) {
    twi_write_stop();
    return(I2C_SDA_NACK); //received NACK on transmit of address
  }

  /* sent the buffer */
  for(i=0; i<len; i++) 
  {
    /* write byte */
    if(!twi_write_byte(buf[i])) {
      twi_write_stop();
      return(I2C_SDA_NACK); //received NACK on transmit of data
    }
  }

  /* if STOP was requested */
  if(sendStop) twi_write_stop();
  
  /* flush potential rubbish */
  twi_flush();
  
  return(I2C_OK);
}

/*****************************************************************
 * software I2C
 * 
 * read from I2C address the number of len bytes in buffer       *
 * **************************************************************/

Wstatus TwCore::twi_readFrom(uint8_t * buf, uint8_t len, uint8_t sendStop) {
    
  uint8_t i;
  
  if(!twi_write_start()) return(I2C_SDA_HELD_LOW_AFTER_INIT);  //line busy

  /* sent address and expect to detect an ACK (low) */
  if(!twi_write_byte(((twSetting.Slave_address << 1) | 1) & 0xFF)) {
    twi_write_stop();
    return (I2C_SDA_NACK);//received NACK on transmit of address
  }

  /* false will cause an a low (ACK) to be written */
  for(i=0; i<(len-1); i++)
    buf[i] = twi_read_byte(false);

  /* A NACK.. just in case client sending more ? */
  buf[len-1] = twi_read_byte(true);

  if(sendStop)  twi_write_stop();
  
  /* flush potential rubbish */
  twi_flush();
  
  return(I2C_OK);
}
/***********************************************************************
 * Read an number of bytes from I2C sending a repeated start after writing
 * the required register. Only works if your device supports this mode
 **********************************************************************/

Wstatus TwCore::twi_readFrom_rs(char* regaddr, uint8_t * buf, uint8_t len) {
  
  uint8_t i;

  /* sent start */
  if(!twi_write_start()) return(I2C_SDA_HELD_LOW_AFTER_INIT);  //line busy

  /*! first sent the slave address and register with write bit (0)
   *  then perform a restart */
  
  /* sent address and expect to detect an ACK (low) */
  if(!twi_write_byte(((twSetting.Slave_address << 1)) & 0xFE)) {
    twi_write_stop();
    return(I2C_SDA_NACK);//received NACK on transmit of address
  }

  /* write register and perform restart and expect an ACK (low) */
  if(!twi_write_byte_rs((regaddr[0] & 0xFF))) {
    twi_write_stop();
    return(I2C_SDA_NACK);     //received NACK on transmit of register
  } 

    /*! sent slave address again with read bit set  (1)*/
   if(!twi_write_byte(((twSetting.Slave_address << 1) | 1) & 0xFF)) {
    twi_write_stop();
    return(I2C_SDA_NACK);//received NACK on transmit of address
  } 
   
  /* false will cause an a low (ACK) to be written after each byte */
  for(i=0; i<(len-1); i++)
    buf[i] = twi_read_byte(false);
  
  /* A NACK.. just in case client sending more ? */
  buf[len-1] = twi_read_byte(true);

  twi_write_stop();
  
  /* flush potential rubbish to follow */
  twi_flush();
  
  return(I2C_OK);
}

/****************************************************************
 * checks line status
 * 
 * return
 *  I2C_OK : no line error detected
 *  I2C_SCL_HELD_LOW : SCL line is held low
 *  I2C_SCL_HELD_LOW_AFTER_READ : SCL is held low for longer than clock stretch
 *  I2C_SDA_HELD_LOW : SDA line is held low
 *  I2C_SDA_HELD_LOW_AFTER_INIT : error during I2C start sequence 
 *****************************************************************/
Wstatus TwCore::twi_status() {
    
    /* SCL held low by another device, no procedure available to recover */
    if (SCL_READ()==0)        return I2C_SCL_HELD_LOW;             
    
    uint32_t clockCount = 20;

    /* if SDA low, read the bits slaves have to sent to a max */
    while (SDA_READ()==0 && clockCount>0) {   
        --clockCount;
        twi_read_bit();
    }

    clockCount = 0;
    
    /* wait for SCL to be high */
    while (SCL_READ() == 0 && (clockCount++) < clck_str_info.clockStretchCount)
        twi_delay(1);  // Clock stretching
    
    /* if clock stretch expired, indicate error
     * I2C bus error. SCL held low beyond slave clock stretch time */
    if (clockCount == clck_str_info.clockStretchCount)
        return I2C_SCL_HELD_LOW_AFTER_READ;  
        
    /* I2C bus error. SDA line held low by slave/another_master after n bits.*/
    if (SDA_READ()==0)
        return I2C_SDA_HELD_LOW;             
    
    /* line busy. SDA again held low by another device. 2nd master? */
    if(!twi_write_start())   return I2C_SDA_HELD_LOW_AFTER_INIT;  
    
    /* all OK */
    return I2C_OK;
}

/*********************************************************************
 * 
 * Initialize the BCM2835 library for the raspberry Pi
 * and set for hardware I2C communication
 *
 *********************************************************************/

Wstatus BcmCore::bcm_init() {
 
    if (!bcm2835_init()) {
        db_print((char *)"Can't init bcm2835!\n");
        return(TW_INTERNAL_ERROR);
    }

    /* will select I2C channel 0 or 1 depending on board reversion. */
    if (!bcm2835_i2c_begin()){
        db_print((char *)"Can't setup i2c pin!\n");
        return(TW_INTERNAL_ERROR);
    }
 
    /* set speed */
    bcm2835_i2c_set_baudrate(twSetting.baudrate *1000);

    return(TW_SUCCESS);
}

/*********************************************************************
 * close the BCM2835 library for the raspberry Pi
 *********************************************************************/

void BcmCore::bcm_close() {
    
    /* reset pins */
    bcm2835_i2c_end();
   
    /* release memory */
    bcm2835_close();
} 

/*********************************************************************
 *
 * hard : write data from buff with length to the I2c device
 * slave address must have been set before with set_slave()
 *
 *********************************************************************/

Wstatus BcmCore::bcm_write(char *buff, uint8_t length) {
    
    int     result;

    /* set slave address */
    bcm2835_i2c_setSlaveAddress(twSetting.Slave_address);

    /* perform a write of data */
    result = bcm2835_i2c_write(buff, length);
    
    switch(result)
    {
        case BCM2835_I2C_REASON_ERROR_NACK :
            db_print((char *)"write NACK error\n");
            return(I2C_SDA_NACK);
    
        case BCM2835_I2C_REASON_ERROR_CLKT :
            db_print((char *)"write Clock stretch error\n");
            return(I2C_SCL_CLKSTR);
    
        case BCM2835_I2C_REASON_ERROR_DATA :
            db_print((char *)"write not all data has been sent\n");
            return(I2C_SDA_DATA);
    
        case BCM2835_I2C_REASON_OK:
            return(I2C_OK);
    
        default :
            db_print((char *)"Unkown error during writing\n");
            return(I2C_SDA_DATA);
    }
}

/*********************************************************************
 *
 * hard : read data from device into buff with length 
 * slave address must have been set before with set_slave()
 *
 *********************************************************************/

Wstatus BcmCore::bcm_read(char *buff, uint8_t length) {
    
    int result;

    /* set slave address */
    bcm2835_i2c_setSlaveAddress(twSetting.Slave_address);
    
    result = bcm2835_i2c_read(buff, length);

    /* process result */
    switch(result)
    {
        case BCM2835_I2C_REASON_ERROR_NACK :
            db_print((char *)"NACK error\n");
            return(I2C_SDA_NACK);

        case BCM2835_I2C_REASON_ERROR_CLKT :
            db_print((char *)"Clock stretch error\n");
            return(I2C_SCL_CLKSTR);
            
        case BCM2835_I2C_REASON_ERROR_DATA :
            db_print((char *)"not all data has been read\n");
            return(I2C_SDA_DATA);
            
        case BCM2835_I2C_REASON_OK:
            return(I2C_OK);
            
        default:
            db_print((char *)"unkown return code\n");
            return(I2C_SDA_DATA);
    }
}


Wstatus BcmCore::bcm_read_rs(char * regaddr, char *buff, uint8_t length) {
    
    int result;

    /* set slave address */
    bcm2835_i2c_setSlaveAddress(twSetting.Slave_address);
    
    result = bcm2835_i2c_read_register_rs(regaddr, buff, length);

    /* process result */
    switch(result)
    {
        case BCM2835_I2C_REASON_ERROR_NACK :
            db_print((char *)"NACK error\n");
            return(I2C_SDA_NACK);

        case BCM2835_I2C_REASON_ERROR_CLKT :
            db_print((char *)"Clock stretch error\n");
            return(I2C_SCL_CLKSTR);
            
        case BCM2835_I2C_REASON_ERROR_DATA :
            db_print((char *)"not all data has been read\n");
            return(I2C_SDA_DATA);
            
        case BCM2835_I2C_REASON_OK:
            return(I2C_OK);
            
        default:
            db_print((char *)"unkown return code\n");
            return(I2C_SDA_DATA);
    }
}

/*********************************************************************
 *
 * display driver / debug message (if requested)
 *
 *********************************************************************/
void db_print(char *mes) {
    if (twSetting.dbmes) printf("%s", mes);
}
