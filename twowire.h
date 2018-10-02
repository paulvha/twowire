/**********************************************************************
*
* some parts are based on original code for ESP8266 from SparkFun
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*
* Distributed as-is; no warranty is given.
* 
* ********************************************************************
* Modified September 2018 Paul van Haastrecht
*  
* It has been adjusted and extended to work on a Raspberry Pi. 
* This library will support BOTH hardware and software I2C communication.
* The software i2C (bit banging) can now be handled on any pin, but 
* also provides support for clock Stretching for slow I2C devices. 
*
* Resources / dependencies:
* BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
*
* Development environment specifics:
* Raspberry Pi / linux Jessie release
* *******************************************************************
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
* along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/


#ifndef __TWOWIRE_H__
#define __TWOWIRE_H__

# include <stdint.h>
# include <stddef.h>
# include <bcm2835.h>

#define BUFFER_LENGTH       100         // tx-buffer
#define default_si2c_clock  100         // 100Khz default
#define default_si2c_stretch 200        // 200us max for clock stretch

/*! 
 * The twi_delay(1) is called during testing for clockstretch()
 * On a Pi-3 a call takes ~1.8 us due to 0.8us program overhead.
 * This has been concluded with extensive testing and measurements 
 * 
 * To set a more accurate uSec time the clockstretch is corrected by 1.8.
 */
#define ClkStrCor   1.8

// capture clock stretch information
struct clock_stretch
{
    uint32_t          clockStretchCount;   // corrected count for software clock stretch
    /*! during reading */
    uint32_t          r_clock_stretch_max;   // maximum clock stretch count
    uint32_t          r_clock_stretch_min;   // min clock stretch count (not zero)
    uint32_t          r_clock_stretch_tot;   // total clock stretch counts
    uint32_t          r_clock_stretch_cnt;   // total counts clock stretch happened
    uint32_t          r_clock_stretch_err;   // clock stretch exceeding limit count
    /*! during writing */
    uint32_t          w_clock_stretch_max;   // maximum clock stretch count
    uint32_t          w_clock_stretch_min;   // min clock stretch count (not zero)
    uint32_t          w_clock_stretch_tot;   // total clock stretch counts
    uint32_t          w_clock_stretch_cnt;   // total counts clock stretch happened
    uint32_t          w_clock_stretch_err;   // clock stretch exceeding limit count
};

struct twowire_s
{
  public:

    unsigned char   twi_sda;                 // pin for software SDA
    unsigned char   twi_scl;                 // pin for software SCL
    uint64_t         twi_dcount;             // down count for software timing
    uint16_t         baudrate;               // clock frequency set
    bool              hw_interface;           // true = use hardware I2C else software
    bool              hw_initialized;         // set if hardware has been initialized
    uint8_t          Slave_address;          // holds the slave address
    bool              dbmes;                  // true = display driver messages
    bool              Pullup;                 // Pullup resistor on SDA/SCL line
};

// define which I2C to use
#define hard_I2C 1
#define soft_I2C 0

// Return values 
typedef enum
{
    TW_SUCCESS,             // 0
    TW_ID_ERROR,            // 1
    TW_I2C_ERROR,           // 2
    TW_INTERNAL_ERROR,      // 3
    TW_GENERIC_ERROR,       // 4
    TW_PARAM_ERROR,         // 5
    I2C_SCL_HELD_LOW,       // 6
    I2C_SCL_HELD_LOW_AFTER_READ,    // 7
    I2C_SDA_HELD_LOW,       // 8
    I2C_SDA_HELD_LOW_AFTER_INIT,    // 9
    I2C_SDA_NACK,           // 10
    I2C_SCL_CLKSTR,         // 11
    I2C_SDA_DATA,           // 12
    I2C_OK                  // 13
    //...
} Wstatus;

/* display driver message if requested */
void db_print(char *mes);

/****************************************************************
 * This is the core operational class of the software I2C.
 * TwCore contains only read and write operations towards the slave.
 * 
 * !!!!!!!//////////////////////////////////////////////// !!!!!
 * !!!!   Do NOT use directly but use the TwoWire routines
 * !!!!!!!//////////////////////////////////////////////// !!!!!
 * 
 ******************************************************************/
class TwCore
{
public:
    /* create constructor */
    TwCore (void);

protected:
    /* initialize for software I2C */
    Wstatus twi_init();
    
    /* set the timing for bit-banging */
    void twi_setClock(uint16_t freq);
    
    /* set the clockstretch limit */
    void twi_setClockStretchLimit(uint32_t limit);
  
    /* get clock stretch statistics */
    void twi_GetStretchStat(clock_stretch *stat);
    
    /* clear clock stretch statistics */
    void twi_ClrStretchStat();
    
    /* get the line status */
    Wstatus twi_status(); 

    /* delay for x microseconds */
    void twi_delay(uint64_t usec);
    
    /* read a bit from the SDA line */
    bool twi_read_bit(void);
    
    /* make the SDA and SCL line floating / input*/
    void twi_stop(void);

    /* read from the slave an amount of bytes */
    Wstatus twi_readFrom(uint8_t * buf, uint8_t len, uint8_t sendStop);
    
    /* read register from slave with repeated start */
    Wstatus twi_readFrom_rs(char* regaddr, uint8_t * buf, uint8_t len);
    
    /* read a single byte from the slave */
    uint8_t twi_read_byte(bool nack);
    
    /* write a buffer to the slave device */
    Wstatus twi_writeTo(uint8_t * buf, uint8_t len, uint8_t sendStop);
    
    /* write a bit sequence on I2C */
    bool twi_write_bit(bool bit);
    
    /* write a byte to the slave device */
    bool twi_write_byte(unsigned char byte);
    
    /* write byte to slave and perform restart */
    bool twi_write_byte_rs(unsigned char byte); 
      
    /* perform an I2C start sequence */
    bool twi_write_start(void);
    
    /* perform an I2C stop sequence */
    bool twi_write_stop(void);

    /* log the clockstretch statistics */
    void twi_LogClockStretchStat(uint32_t count, int err, bool act);
    
    /* flush rubbish */
    void twi_flush();
};

/******************************************************************
 * BcmCore contains the calls to perform a hardware I2C
 * the BCM2835 internal i2C hardware will handle the communication
 * with the I2C slave device
 * 
 * !!!!!!!//////////////////////////////////////////////// !!!!!
 * !!!!! Do NOT use directly but use the TwoWire routines
 * !!!!!!!//////////////////////////////////////////////// !!!!!
 *****************************************************************/
class BcmCore 
{
public:
    /* create constructor */
    BcmCore(void);
    
    /* initialize the BCM2835 for I2C communication */
    Wstatus bcm_init();
    
    /* reset the BCM2835 and release memory */
    void bcm_close();
    
    /* sent a buffer of bytes to the slave */
    Wstatus bcm_write(char *buff, uint8_t length);
    
    /* read a buffer of bytes from the slave */
    Wstatus bcm_read(char *buff, uint8_t length);
    
    /* read a buffer of bytes from a register of the slave
     * using repeated start function. */
    Wstatus bcm_read_rs(char * regaddr, char *buff, uint8_t length); 

private:
};

/*********************************************************************
 * This is the highest level class of the driver.
 *
 * class TwoWire inherits the TwCore and makes use of the begin()
 * method through its own begin() method. 
 * 
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!! A user program should ONLY use this TwoWire API for accessing.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * 
 ********************************************************************/
 
class TwoWire : public TwCore, BcmCore
{
public:

    /* create constructor */
    TwoWire(void);

    /* enable/disable messages from driver: true or false(default) */
    void setDebug(bool db);
    
    /* set for internal pullup resistor on the SDA & SCL GPIO */
    void setPullup();
     
    /* Initialize the I2C communicator 
     * 
     * Itoc: soft_I2C for software I2C 
     * software is bit-banging and SDA and SCL can be set to any GPIO. 
     * this can also handle clock stretching by the slave correctly
     * 
     * Itoc: hard_I2C is handled by the BCM2835. (default) 
     * The SDA and SCL GPIO are fixed. Will handle clock stretching 
     * badly due to BCM2835 chip 
     */
    Wstatus begin(bool itoc = hard_I2C, int sda = 2, int scl = 3);
    
    /* set the I2C slave device address to use */
    void setSlave(uint8_t address);
     
    /* close the I2C communicator correctly */
    void close();
    
    /* write a buffer of data to the slave device, using the selected 
     * I2C communicator (hardware or software)
     */
    Wstatus i2c_write(char *buff, uint8_t length);
    
    /* read into a buffer data from the slave device, using the selected 
     * I2C communicator (hardware or software)
     */
    Wstatus i2c_read(char *buff, uint8_t length);
    
    /* read into a buffer data from a register of a slave device. This 
     * will be done with repeated start. Some slave devices require this */
    Wstatus i2c_read_rs(char * regaddr, char *buff, uint8_t length);
    
    /* check whether the SDA and SCL lines are in the correct status 
     * return :
     * 
     *  I2C_OK : no line error detected
     *  I2C_SCL_HELD_LOW : SCL line is held low
     *  I2C_SCL_HELD_LOW_AFTER_READ : SCL is held low for longer than clock stretch
     *  I2C_SDA_HELD_LOW : SDA line is held low
     *  I2C_SDA_HELD_LOW_AFTER_INIT : error during I2C start sequence 
     */
    Wstatus linestatus();
    
    /* set the I2C speed between 1 and 400 Khz  (100Khz is default)*/
    void setClock(uint16_t frequency);
    
    /***************************************************************
     * following commands are only valid for software I2C
     **************************************************************/
    
    /* set the clock stretch in us (200 us is default)
     * only used for software I2c. It will clear the statistics
     */
    void setClockStretchLimit(uint32_t limit);
    
    /* get clock stretch statistics since last clear */
    void GetStretchStat(clock_stretch *stat);
    
    /* clear clock stretch statistics */
    void ClrStretchStat();
    
    /* display clock stretch statistics */
    void DispClockStretch();
    
    /* flush any of the send buffers */
    void flush(void);

private:

    /* set as sending and set tx pointers correctly */
    void beginTransmission();
    
    /* perform the sending of tx buffer to the slave */
    Wstatus endTransmission(uint8_t sendStop);
    Wstatus endTransmission(void);
    
    /* read buffer of bytes from slave */ 
    size_t requestFrom( size_t size, bool sendStop);
    
    /* write single byte to slave */
    size_t write(char data);
    
    /* write buffer of bytes to slave */
    size_t write(char *data, uint8_t quantity);
};

#endif  // End of definition check
