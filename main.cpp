/************************************
 * FDC2214 Touchless Buttons
 * Rev 1 - shabaz - May 2020
 *
 * Connections:
 * FRDM-KL25Z:
 * PTC11 - SDA  // was PTE25 on K64F
 * PTC10 - SCL  // PTE24
 * PTC0 - shut
 * PTC3 - addr
 * PTC4 - intb
 *
 ***********************************/

// defines
#include "mbed.h"
#include <ctype.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#define INTB_PIN PTC4

#define MTSTART 8000

// optional low-pass filtering. Not needed! So set to 1.
#define BSIZE 1

// buffer sizes
#define MSIZE 16
#define LSIZE 32

// number of buttons, 1-4
#define BNUM 4

// LCD registers and details
// If your LCD has 4 full digits (i.e. 8.8.8.8), then set DISP_TYPE to 4 
// If the LCD has only 3 full digits (i.e. 1.8.8.8) then set to 3
// Only value 3 has been tested so far.
#define DISP_TYPE 3
// LCD Commands (these can be 'continued' by setting the MSB
#define LCD_DEVICE_SUBADDR 0x60
#define BLINK_OFF 0x70
#define BANK_CMD 0x78
// LCD BANK_CMD bit values
#define WR_BANK_A 0x00
#define WR_BANK_B 0x02
#define DISP_BANK_A 0x00
#define DISP_BANK_B 0x01
// LCD I2C addr (8-bit)
#define LCD_I2C_ADDR (0x38<<1)

/*********************************
 * FDC2214 register addresses
 *********************************/
// Converter output data registers (there is a formula to convert to
// actual capacitance values
#define DATA_MSW_CH0 0x00
#define DATA_LSW_CH0 0x01
#define DATA_MSW_CH1 0x02
#define DATA_LSW_CH1 0x03
#define DATA_MSW_CH2 0x04
#define DATA_LSW_CH2 0x05
#define DATA_MSW_CH3 0x06
#define DATA_LSW_CH3 0x07
// reference count registers (affects conversion time)
#define CMD_RCOUNT_CH0 0x08
#define CMD_RCOUNT_CH1 0x09
#define CMD_RCOUNT_CH2 0x0a
#define CMD_RCOUNT_CH3 0x0b
// offset registers (allows for higher-res readings when gain is set to higer than 1)
#define CMD_OFFSET_CH0 0x0c
#define CMD_OFFSET_CH1 0x0d
#define CMD_OFFSET_CH2 0x0e
#define CMD_OFFSET_CH3 0x0f
// oscillator stabilization time registers
#define CMD_SETTLECOUNT_CH0 0x10
#define CMD_SETTLECOUNT_CH1 0x11
#define CMD_SETTLECOUNT_CH2 0x12
#define CMD_SETTLECOUNT_CH3 0x13
// these set up the reference clock division to suit the tank circuit oscillator
// frequency and single- or differential-ended configuration
#define CMD_CLOCK_DIVIDERS_CH0 0x14
#define CMD_CLOCK_DIVIDERS_CH1 0x15
#define CMD_CLOCK_DIVIDERS_CH2 0x16
#define CMD_CLOCK_DIVIDERS_CH3 0x17
// these need to be set so that the oscillation amplitude falls within a certain range
#define CMD_DRIVE_CURRENT_CH0 0x1e
#define CMD_DRIVE_CURRENT_CH1 0x1f
#define CMD_DRIVE_CURRENT_CH2 0x20
#define CMD_DRIVE_CURRENT_CH3 0x21
// CMD_ERROR_CONFIG is used to set up what things you want to be notified about via the interrupt pin
#define CMD_ERROR_CONFIG 0x19
// CMD_MUX_CONFIG defines what channels are scanned, and a deglitch filter bandwidth:
#define CMD_MUX_CONFIG 0x1b
// CMD_RESET_DEV is used to reset the device and to program the gain
#define CMD_RESET_DEV 0x1c
// CMD_CONFIG is used to choose the channel if CMD_MUX_CONFIG is set to not multiplex multiple channels.
// this register is also used to bring the device out of standby mode
#define CMD_CONFIG 0x1a
// device status reporting:
#define CMD_STATUS 0x18
// dummy register value, use to identify end-of-configuration strings in this code
#define CMD_FIN 0xff

// the LED is wired to be switched on when the pin is high
#define LED_ON 0
#define LED_OFF 1

/************* typedef structs *********************/
// LCD related
typedef struct char_prog_s
{
  unsigned char byte0;
  unsigned char byte1;
  unsigned char byte2;
} char_prog_t;
typedef struct ascii_s
{
  char_prog_t char0;
  char_prog_t char1;
  char_prog_t char2;
} ascii_t;

/***********************************
 * global variables
 ***********************************/
 unsigned int MT=MTSTART;
// LCD related; bits for displaying numbers 0-9
const char bit_table[]=
{
  0xf6, 0xc0, 0x6e, 0xea, 0xd8, 0xba, 0xbe, 0xe0, 0xfe, 0xfa
};
// FDC2214 I2C address
const int fdcaddr = (0x2a<<1); // when addrpin=0
// transform variables
const static arm_cfft_instance_f32 *S;
float samples[LSIZE];
float magnitudes[MSIZE];
// LED object
DigitalOut myled(LED1);
// FDC2214 pins
DigitalOut shut(PTC0);
DigitalOut addrpin(PTC3);
DigitalIn intb(INTB_PIN);

// LCD
char lcdbuf[10]; // LCD driver registers buffer
char lcdtxt[5]; // text to display

// debug
Serial pc(USBTX, USBRX);

I2C i2c(PTC11, PTC10);

// FDC2214 configuration params have two bytes, msb and lsb here refer to bytes, not bits : )
typedef struct cfg_param_s
{
    char c_addr;
    char c_param_msb;
    char c_param_lsb;
} cfg_param_t;
// Take these settings from TI's eval board PC software in Menu->Registers window
cfg_param_t cfg_arr[]={
    {CMD_RCOUNT_CH0, 0x20, 0x00},
    {CMD_RCOUNT_CH1, 0x20, 0x00},
    {CMD_RCOUNT_CH2, 0x20, 0x00},
    {CMD_RCOUNT_CH3, 0x20, 0x00},
    {CMD_OFFSET_CH0, 0x00, 0x00},
    {CMD_OFFSET_CH1, 0x00, 0x00},
    {CMD_OFFSET_CH2, 0x00, 0x00},
    {CMD_OFFSET_CH3, 0x00, 0x00},
    {CMD_SETTLECOUNT_CH0, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH1, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH2, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH3, 0x04, 0x00},
    {CMD_CLOCK_DIVIDERS_CH0, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH1, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH2, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH3, 0x10, 0x01},
    {CMD_DRIVE_CURRENT_CH0, 0xf8, 0x00},
    {CMD_DRIVE_CURRENT_CH1, 0xf8, 0x00},
    {CMD_DRIVE_CURRENT_CH2, 0xf8, 0x00},
    {CMD_DRIVE_CURRENT_CH3, 0xf8, 0x00},
    {CMD_ERROR_CONFIG, 0x00, 0x01},
    {CMD_MUX_CONFIG, 0xc2, 0x0d},
    {CMD_CONFIG, 0x1e, 0x01},
    {CMD_FIN, 0xff, 0xff}};

/************ function prototypes ************/
void lcd_init(void);
void lcd_print_line(char* line);

/********************************************
 * classes
 ********************************************/
// data analyzer
class Analyzer {
public:
    Analyzer(void)
    {
        mInh=1;
        mTot=0;
        mCount=0;
        mLCount=0;   
        mActLevel=0;
    }
    // plug in data so that a moving historical window can be processed
    void insert(unsigned int val)
    {
        int i;
        // store raw data
        for (i=1; i<BSIZE; i++)
        {
            mBuf[i-1]=mBuf[i];
        }
        mBuf[BSIZE-1]=val;
        // mTot implements a simple moving average (if required)
        mTot=mTot+val;
        mCount++;
        if (mCount>=BSIZE)
        {
            mCount=0; // mytest
            // store the averaged values into another array, to implement a long-term average
            for (i=1; i<LSIZE; i++)
            {
                mLta[i-1]=mLta[i];
            }
            mLta[LSIZE-1]=mTot/BSIZE;
            //pc.printf("** %d\n", mTot/BSIZE);
            mTot=0;
            mLCount++;
            // is the long-term array now fully populated?
            if (mLCount>=LSIZE)
            {
                // don't inhibit analysis any longer
                mInh=0;
                // set to zero if you don't want to process so frequently
                mLCount=254; 
                // should move this out to a separate function, shouldn't really be in
                // 'insert'
                // we need floats for the transform
                for (i=0; i<LSIZE; i++)
                {
                    samples[i]=(float)mLta[i];
                }
                arm_cfft_f32(S, samples, 0, 1); // < match MSIZE, i.e. half of LSIZE
                arm_cmplx_mag_f32(samples, magnitudes, MSIZE);
                
                mActLevel=(unsigned int)magnitudes[MSIZE-1];
                //pc.printf("%d\n\r", mActLevel);
            }
        }
    }
    // these long-term average and short-term average get functions are not used
    // but left them there for general use of this class
    unsigned int getLta(void)
    {
        int i;
        unsigned int lta=0;
        if (mInh) return 0;
        for (i=0; i<LSIZE; i++)
        {
            lta+=mLta[i];
        }
        lta=lta/LSIZE;
        return(lta);
    }
    unsigned int getSta(void)
    {
        int i;
        unsigned int sta=0;
        if (mInh) return 0;
        for (i=0; i<BSIZE; i++)
        {
            sta+=mBuf[i];
        }
        sta=sta/BSIZE;
        return(sta);
    }        
    unsigned int getActLevel(void)
    {
        return mActLevel;
    }
private:
    unsigned int mBuf[BSIZE];
    unsigned int mLta[LSIZE];
    char mCount;
    char mLCount;
    unsigned int mTot;
    char mInh;
    unsigned int mActLevel;
};

// FDC2214 handler
class Fdc {
public:
    Fdc(void)
    { 
        mEnabled=0;
    }

    void enable()
    {
        mEnabled=1;
    }
    
    void disable()
    {
        mEnabled=0;
    }
    
    void reset()
    {
        char cmd[3];
        // reset FDC2214
        cmd[0]=CMD_RESET_DEV;
        cmd[1]=0x84; // output gain set to 8
        cmd[2]=0x00;
        i2c.write(fdcaddr, cmd, 3);
        wait(0.01);
    }

    void read()
    {
        char cmd[2];
        if (mEnabled)
        {
            // read channel 0
            cmd[0]=DATA_MSW_CH0;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mMsw[0]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH0;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mLsw[0]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
        
            // read channel 1 although the result isn't needed for this project
            cmd[0]=DATA_MSW_CH1;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mMsw[1]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH1;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mLsw[1]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            
            // read ch3 and ch4 
            cmd[0]=DATA_MSW_CH2;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            mMsw[2]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH2;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            mLsw[2]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            
            cmd[0]=DATA_MSW_CH3;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            mMsw[3]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH3;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            mLsw[3]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            
            // read status register to clear interrupt pin
            cmd[0]=CMD_STATUS;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
        
        }
        else
        {
            pc.printf("FDC object not enabled yet\n\r");
        }
    }
    // placeholder. We don't require the actual freq in Hz, nor the capacitance in Farads, for this project
    int getfreq(char chan)
    {
        double afreq;
        unsigned int tfreq;
        tfreq=(mMsw[chan]<<16) | (mLsw[chan]&0xffff);
        printf("tfreq: %d\n\r", tfreq);
        // todo
        return(0);
    }
    // retrieve the FDC capacitance to digital conversion data register value (i.e. raw value)
    unsigned int getDat(char chan)
    {
        unsigned int tdat;
        tdat=(mMsw[chan]<<16) | (mLsw[chan]&0xffff);
        return(tdat);
    }

private:
    unsigned char mEnabled;
    //InterruptIn mInterrupt;
    volatile unsigned int mMsw[4];
    volatile unsigned int mLsw[4];
};

// create the objects
Fdc fdc;
Analyzer analyzer[BNUM];

/*************************************
 * function prototypes
 *************************************/
 
 /************************************
 * functions
 *************************************/
// debugger
void callback()
{
    char c;
    c=pc.getc();
}

/* lcd_init
 *  sets up the buffer for the 3.5 digit LCD described here: https://bit.ly/2PlxXuA
 */
void lcd_init(void)
{
  char mode, device, bank, blinkmode;
  mode = 0xc9; // static mode, display enabled, continuation enabled
  device = LCD_DEVICE_SUBADDR | 0x80; // select the LCD device, continuation enabled
  bank = BANK_CMD | 0x80 | WR_BANK_A | DISP_BANK_A; 
  blinkmode=BLINK_OFF | 0x80;
  lcdbuf[0]=mode;
  lcdbuf[1]=device;
  lcdbuf[2]=blinkmode;
  lcdbuf[3]=bank;
  lcdbuf[4]=0x00; // pointer
}

/* print_line
 *  displays content on the 3.5 digit LCD screen
 */
void lcd_print_line(char* line)
{
  char line_idx=0;
  char lcd_idx=0;
  char not_finished=1;
  char c;
  char byte0=0;
  char byte1=0;
  char byte2=0;
  char byte3=0;
  char byte4=0;

  while(not_finished)
  {
    c=line[line_idx];
    switch(c)
    {
      case '.':
        // handle decimal point
        if (lcd_idx==1)
        {
          byte1 |= 0x01;
        }
        else if (lcd_idx==2)
        {
          byte2 |= 0x01;
        } 
        else if (lcd_idx==3)
        {
          byte3 |= 0x01;
        }
        break;
      case ' ':
        lcd_idx++;
        if (lcd_idx>3)
        {
          not_finished=0;
        }
        break;
      case '\0':
        // line finished
        not_finished=0;
        break;
      default:
        if ((c<'0') || (c>'9'))
        {
          // we can't handle this char
        }
        else
        {
           if (lcd_idx==0)
           {
             if (DISP_TYPE==4)
             {
                byte1 |= bit_table[c-'0'];
             }
             else
             {
               if (c=='1')
               {
                  byte0 |= 0x80;
               }
               else
               {
                  // can't handle any other char
               }
             }
           }
           else if (lcd_idx==1)
           {
             byte2 |= bit_table[c-'0'];
           }
           else if (lcd_idx==2)
           {
             byte3 |= bit_table[c-'0'];
           }
           else if (lcd_idx==3)
           {
              byte4 |= bit_table[c-'0'];
           }
           lcd_idx++;
           if (lcd_idx>3)
           {
             // we're done
             not_finished=0;
           }
        }
        break;
    } // end switch(c)
    line_idx++;
  } // end while(not_finished)

  lcdbuf[5]=byte0;
  lcdbuf[6]=byte1;
  lcdbuf[7]=byte2;
  lcdbuf[8]=byte3;
  lcdbuf[9]=byte4;
  
  i2c.write(LCD_I2C_ADDR, lcdbuf, 10);
}

// main() runs in its own thread in the OS
int
main(void)
{
    char cmd[10];
    unsigned int dat[BNUM];
    unsigned int act[BNUM];
    char evt[4];
    char dstat[4];
    char maxidx;
    unsigned int maxreading;
    unsigned int oldact[BNUM]; // crude output filter. good enough!
        
    myled=LED_OFF; // switch off the LED
  
    int not_finished=1;
    int i=0;
    
    // set up debugger
    pc.baud(115200);
    pc.attach(&callback);
    pc.printf("Hello\n\r");
    
    i2c.frequency(100000);
  
    wait(0.1); // LCD needs a few msec
    lcd_init();
    for (i=0; i<5; i++) {
        lcdtxt[i]=' ';
    }
    lcd_print_line(lcdtxt);

    for (i=0; i<BNUM; i++) {
        oldact[i]=0;
        evt[i]=0;
        dstat[i]=0;
    }

    // transform stuff
    S = &arm_cfft_sR_f32_len16;
    
    // handle FDC2214 startup
    shut=1; // put FDC2214 into shutdown
    addrpin=0; // clear I2C address pin
    wait(0.1);
    shut=0; // bring FDC2214 out of shutdown mode
    wait(0.01);
    // reset FDC2214
    fdc.reset();

    // handle FDC2214 configuration
    not_finished=1;
    i=0;
    do
    {
        cmd[0]=cfg_arr[i].c_addr;
        cmd[1]=cfg_arr[i].c_param_msb;
        cmd[2]=cfg_arr[i].c_param_lsb;
        i2c.write(fdcaddr, cmd, 3);
        i++;
        if (cfg_arr[i].c_addr == CMD_FIN)
        {
            not_finished=0;
            break;
        }
    } while(not_finished);
    pc.printf("cfg done\n\r");
    fdc.enable();
    
    // main forever loop
    while(1)
    {
        if (!intb) // conversions ready?
        {
            // acquire readings for all four buttons
            fdc.read();
            // push the readings into the analyzer object
            for (i=0; i<BNUM; i++) {
                dat[i]=fdc.getDat(i);
                analyzer[i].insert(dat[i]); // plug in the data
                act[i]=analyzer[i].getActLevel();
            }
            
            // in this applications, there's a possibility that more than one
            // reading could increase when a user has big hands
            maxidx=0;
            maxreading=0;

            for (i=0; i<BNUM; i++) {
                if (act[i]>MT) {
                    if (act[i]>maxreading) {
                        maxreading=act[i];
                        maxidx=i;
                    }
                }
            }
            
            // test to see if we have sufficient hand-swipe activity.
            // 4000 worked fine but so did 3000. It's not a super critical value
            for (i=0; i<BNUM; i++) {
                if ((act[i]>MT) && (oldact[i]>MT))
                {
                    if (i!=maxidx)
                        continue;
                    //evt implements a one-shot trigger
                    if (evt[i]==0)
                    {
                        evt[i]=1;
                        // dstat implements a toggle action
                        if (dstat[i]==0)
                        {
                            dstat[i]=1;  
                            myled=LED_ON;
                            lcdtxt[i]=i+1+'0';
                            lcd_print_line(lcdtxt);
                        }
                        else
                        {
                            dstat[i]=0;
                            myled=LED_OFF;
                            lcdtxt[i]=' ';
                            lcd_print_line(lcdtxt);
                        }
                    }    
                }
                else if ((act[i]<=MT) && (oldact[i]<=MT))
                {
                    //clear the one-shot trigger
                    evt[i]=0;
                }
                oldact[i]=act[i];
            }
        }
    }
    return(0); // warning on this line is ok
}

