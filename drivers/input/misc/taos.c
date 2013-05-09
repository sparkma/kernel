/*******************************************************************************
*                                                                              *
*   File Name:    taos.c                                                      *
*   Description:   Linux device driver for Taos ambient light and         *
*   proximity sensors.                                     *
*   Author:         John Koshi                                             *
*   History:   09/16/2009 - Initial creation                          *
*           10/09/2009 - Triton version         *
*           12/21/2009 - Probe/remove mode                *
*           02/07/2010 - Add proximity          *
*                                                                              *
********************************************************************************
*    Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
// includes
//[ECID:000000] ZTEBSP wanghaifei start 20111018, remove some include
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
//#include <linux/hwmon.h>
//#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
//#include <asm/delay.h>
#include <linux/taos_common.h>
#include <linux/delay.h>
//iVIZM
#include <linux/irq.h> 
#include <linux/interrupt.h> 
#include <linux/slab.h>
//#include <mach/gpio.h> 
//#include <linux/poll.h> 
#include <linux/wakelock.h>
#include <linux/input.h>
//[ECID:000000] ZTEBSP wanghaifei end 20111018


// device name/id/address/counts
#define TAOS_DEVICE_NAME                "taos"
//#define TAOS_DEVICE_ID                  "tritonFN"
#define TAOS_DEVICE_ID                  "taos"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_TRITON_MAXREGS             32
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                8

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x10 //iVIZM
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20

// lux constants
#define TAOS_MAX_LUX                    10000
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3
#define CHIP_ID                         0x3d


extern bool in_call_state(void); //[ECID:000000] ZTEBSP wanghaifei 201203019, for in call suspend
// forward declarations
static int taos_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
static int taos_remove(struct i2c_client *client);

static int taos_open(struct inode *inode, struct file *file);
static int taos_release(struct inode *inode, struct file *file);
//[ECID:000000] ZTEBSP wanghaifei start 20111018, remove ioctl since 2.6.36, remove taos_read,taos_write,taos_llseek 
//static int taos_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static long taos_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
//static int taos_read(struct file *file, char *buf, size_t count, loff_t *ppos);
//static int taos_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
//static loff_t taos_llseek(struct file *file, loff_t offset, int orig);
//[ECID:000000] ZTEBSP wanghaifei end 20111018

static int taos_get_lux(void);
static int taos_lux_filter(int raw_lux);
static int taos_prox_poll(struct taos_prox_info *prxp);
/*[ECID:000000] ZTEBSP wanghaifei start 20111020,  remove taos_read,taos_write,taos_llseek */
/*
static void taos_prox_poll_timer_func(unsigned long param);
static void taos_prox_poll_timer_start(void);
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/
//iVIZM
static int taos_als_threshold_set(void);
static int taos_prox_threshold_set(void);
static int taos_als_get_data(void);
static int taos_interrupts_clear(void);
static int taos_resume(struct i2c_client *client);
static int taos_suspend(struct i2c_client *client,pm_message_t mesg);

/*[ECID:000000] ZTEBSP wanghaifei start 20111020, remove some code useless*/
/*
DECLARE_WAIT_QUEUE_HEAD(waitqueue_read);//iVIZM

#define ALS_PROX_DEBUG //iVIZM
static unsigned int ReadEnable = 0;//iVIZM
struct ReadData { //iVIZM
    unsigned int data;
    unsigned int interrupt;
};
struct ReadData readdata[2];//iVIZM
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/

// first device number
static dev_t taos_dev_number;


// class structure for this device
static struct class *taos_class; //[ECID:000000] ZTEBSP wanghaifei 20111020, modify to static

// module device table
static struct i2c_device_id taos_idtable[] = {
    {TAOS_DEVICE_ID, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);


// client and device
//iVIZM
static char pro_buf[4]; //iVIZM
static char als_buf[4]; //iVIZM
static int ALS_ON;

// driver definition
static struct i2c_driver taos_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = TAOS_DEVICE_NAME,
    },
    .id_table = taos_idtable,
    .probe = taos_probe,
    .resume = taos_resume,//iVIZM
    .suspend = taos_suspend,//iVIZM
    .remove = __devexit_p(taos_remove),
};

// per-device data
//[ECID:000000] ZTEBSP wanghaifei 20111020, remove some variable unsless in struct taos_data
struct taos_data {
    struct i2c_client *client;
    struct cdev cdev;
    struct input_dev *input_dev;//iVIZM
    struct work_struct work;//iVIZM
    struct wake_lock taos_wake_lock;//iVIZM
    char taos_name[TAOS_ID_NAME_SIZE];
    int working;
    int open_num;
} *taos_datap;
//[ECID:000000] ZTEBSP wanghaifei end 20111020

// file operations
static struct file_operations taos_fops = {
    .owner = THIS_MODULE,
    .open = taos_open,
    .release = taos_release,
/*[ECID:000000] ZTEBSP wanghaifei start 20111018, use unlocked_ioctl*/
/*
    .read = taos_read,
    .write = taos_write,
    .llseek = taos_llseek,
    .ioctl = taos_ioctl,
*/
    .unlocked_ioctl = taos_unlocked_ioctl,
/*[ECID:000000] ZTEBSP wanghaifei end 20111018*/
   // .poll = taos_poll, //iVIZM
};

// device configuration
struct taos_cfg *taos_cfgp;
static u32 calibrate_target_param = 300000;
static u16 als_time_param = 200;        //0.2*3 = 0.6
static u16 scale_factor_param = 1;
static u16 gain_trim_param = 512;
static u8 filter_history_param = 3;
static u8 filter_count_param = 3;             //0x0c
static u8 gain_param = 1;   //modified by fanjiankang for gain
static u16 prox_threshold_hi_param = 0x230;
static u16 prox_threshold_lo_param = 0x218;
static u16 als_threshold_hi_param = 3000;//iVIZM
static u16 als_threshold_lo_param = 10;//iVIZM
static u8 prox_int_time_param = 0xEE;//50ms
static u8 prox_adc_time_param = 0xFF;
static u8 prox_wait_time_param = 0xEE;
static u8 prox_intr_filter_param = 0x23;
static u8 prox_config_param = 0x00;
static u8 prox_pulse_cnt_param = 0x04; //[ECID:000000] ZTEBSP wanghaifei[via DangXiao] 20120116, reduce emit power
static u8 prox_gain_param = 0x61;

// prox info
struct taos_prox_info prox_cal_info[20];
struct taos_prox_info prox_cur_info;
struct taos_prox_info *prox_cur_infop = &prox_cur_info;
//static struct timer_list prox_poll_timer;//[ECID:000000] ZTEBSP wanghaifei 20111020, remove variable useless
static int prox_on = 0;
//static int device_released = 0; //[ECID:000000] ZTEBSP wanghaifei 20111020, remove variable useless
static u16 sat_als = 0;
static u16 sat_prox = 0;

// device reg init values
u8 taos_triton_reg_init[16] = {0x00,0xFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00};

// lux time scale
struct time_scale_factor  {
    u16 numerator;
    u16 denominator;
    u16 saturation;
};
struct time_scale_factor TritonTime = {1, 0, 0};
struct time_scale_factor *lux_timep = &TritonTime;

// gain table
u8 taos_triton_gain_table[] = {1, 8, 16, 120};

// lux data
struct lux_data {
    u16 ratio;  //红外线比上全部域的光线
    u16 clear;; //红外线比上全部域的光线
    u16 ir;        //光线中的红外线

};
struct lux_data TritonFN_lux_data[] = {
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;
static int lux_history[TAOS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};//iVIZM

static irqreturn_t taos_irq_handler(int irq, void *dev_id) //iVIZM
{
    schedule_work(&taos_datap->work);

    return IRQ_HANDLED;
}

static int taos_get_data(void)//iVIZM
{
    int ret = 0;
    int status; //[ECID:000000] ZTEBSP wanghaifei 20111020, add variable

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | 0x13)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte(1) failed in taos_work_func()\n");
        return (ret);
    }
    status = i2c_smbus_read_byte(taos_datap->client);
    if((status & 0x20) == 0x20) {
        ret = taos_prox_threshold_set();
/*
        if(ret >= 0)
            ReadEnable = 1;
*/
    } else if((status & 0x10) == 0x10) {
       // ReadEnable = 1;
        taos_als_threshold_set();
        taos_als_get_data();
    }
    return ret;
}

static int taos_interrupts_clear(void)//iVIZM
{
    int ret = 0;
    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte(2) failed in taos_work_func()\n");
        return (ret);
    }
    return ret;
}
static void taos_work_func(struct work_struct * work) //iVIZM
{
    wake_lock(&taos_datap->taos_wake_lock);
    taos_get_data();
    taos_interrupts_clear();
    wake_unlock(&taos_datap->taos_wake_lock);
}
static int taos_als_get_data(void)//iVIZM
{
    int ret = 0;
    u8 reg_val;
    int lux_val = 0;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);
    if (((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL ))| TAOS_TRITON_CNTL_PWRON) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
    {
	return -ENODATA;
    }
    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);
    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
    {
	return -ENODATA;
    }
    if ((lux_val = taos_get_lux()) < 0)
        printk(KERN_ERR "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val);
 //   lux_val = taos_lux_filter(lux_val);
//    printk("********** before taos_als_get_data ********* lux_val = %d \n",lux_val);
	input_report_abs(taos_datap->input_dev,ABS_MISC,lux_val);
	input_sync(taos_datap->input_dev);//[ECID:000000] ZTEBSP wanghaifei 20111019, input SYNC
    return ret;
}

static int taos_als_threshold_set(void)//iVIZM
{
    int i,ret = 0;
    u8 chdata[2];
    u16 ch0;
    int mcount; //[ECID:000000] ZTEBSP wanghaifei 20111020, add variable

    for (i = 0; i < 2; i++) {
        chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW | (TAOS_TRITON_ALS_CHAN0LO + i))));
    }
    ch0 = chdata[0] + chdata[1]*256;
    als_threshold_hi_param = (12*ch0)/10;
	if (als_threshold_hi_param >= 65535)
        als_threshold_hi_param = 65535;	 
    als_threshold_lo_param = (8*ch0)/10;
    als_buf[0] = als_threshold_lo_param & 0x0ff;
    als_buf[1] = als_threshold_lo_param >> 8;
    als_buf[2] = als_threshold_hi_param & 0x0ff;
    als_buf[3] = als_threshold_hi_param >> 8;

    for( mcount=0; mcount<4; mcount++ ) { 
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x04) + mcount, als_buf[mcount]))) < 0) {
             printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos als threshold set\n");
             return (ret);
        }
    }
    return ret;
}

static int taos_prox_threshold_set(void)//iVIZM
{
	int i,ret = 0;
	u8 chdata[6];
	u16 proxdata = 0;
	u16 cleardata = 0;
	int data = 0;
	int mcount; //[ECID:000000] ZTEBSP wanghaifei 20111020, add variable

	for (i = 0; i < 6; i++) {
		chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW| (TAOS_TRITON_ALS_CHAN0LO + i))));
	}
	cleardata = chdata[0] + chdata[1]*256;
	proxdata = chdata[4] + chdata[5]*256;
	if (prox_on || proxdata < taos_cfgp->prox_threshold_lo ) {
		pro_buf[0] = 0x0;
		pro_buf[1] = 0x0;
		pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0ff;
		pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;
		data = 0;
		input_report_abs(taos_datap->input_dev,ABS_DISTANCE,data);
	} else if (proxdata > taos_cfgp->prox_threshold_hi ){
		if (cleardata > ((sat_als*80)/100))
			return -ENODATA;
		pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
		pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
		pro_buf[2] = 0xff;
		pro_buf[3] = 0xff;
		data = 1;
		input_report_abs(taos_datap->input_dev,ABS_DISTANCE,data);
    }
    input_sync(taos_datap->input_dev);//[ECID:000000] ZTEBSP wanghaifei 20111019, input SYNC


    for( mcount=0; mcount<4; mcount++ ) { 
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x08) + mcount, pro_buf[mcount]))) < 0) {
             printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos prox threshold set\n");
             return (ret);
        }
    }
	
    prox_on = 0;
    return ret;
}

// driver init
static int __init taos_init(void) {
    int ret = 0;

    if ((ret = (i2c_add_driver(&taos_driver))) < 0) {
        printk(KERN_ERR "TAOS: i2c_add_driver() failed in taos_init()\n");
    }

    return (ret);
}

// driver exit
static void __exit taos_exit(void) {
/*[ECID:000000] ZTEBSP wanghaifei start 20111018, move disable_irq to taos_open ,change exit sequence */
/*
    i2c_del_driver(&taos_driver);
    unregister_chrdev_region(taos_dev_number, TAOS_MAX_NUM_DEVICES);
    device_destroy(taos_class, MKDEV(MAJOR(taos_dev_number), 0));
    cdev_del(&taos_datap->cdev);
    class_destroy(taos_class);
   disable_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,ALS_PS_GPIO-1));
    kfree(taos_datap);
*/
    free_irq(taos_datap->client->irq, NULL);
    kfree(taos_cfgp);
    input_unregister_device(taos_datap->input_dev);
    device_destroy(taos_class, MKDEV(MAJOR(taos_dev_number), 0));
    class_destroy(taos_class);
    cdev_del(&taos_datap->cdev);
    kfree(taos_datap);
    unregister_chrdev_region(taos_dev_number, TAOS_MAX_NUM_DEVICES);
    i2c_del_driver(&taos_driver);
/*[ECID:000000] ZTEBSP wanghaifei end 20111018 */
}

// client probe
static int taos_probe(struct i2c_client *clientp, const struct i2c_device_id *idp) {
    int ret = 0;
    int chip_id;
    struct taos_platform_data* tdata;

    printk( "TAOS:  taos_probe begin\n");

    tdata =  (struct taos_platform_data *) clientp->dev.platform_data;
    if (!tdata) {
	    dev_WARN(&clientp->adapter->dev,
			    "Missing platform data for taos\n");
    } else {
	    tdata->init_hw_power();
	    mdelay(10);
    }


    if ((ret = (i2c_smbus_write_byte(clientp, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + TAOS_TRITON_CHIPID))))) < 0) {
	    printk(KERN_ERR "TAOS: i2c_smbus_write_byte() to chipid reg failed in taos_probe\n");
	    return -ENODATA;
    }
    chip_id = i2c_smbus_read_byte(clientp);
//[ECID:000000] ZTEBSP wanghaifei start 20111125, support TMD27711 and TMD27713
    printk("taos chip id is:0x%x\n",chip_id);
    if((chip_id != 0x20) && (chip_id != 0x29) ) {
	    printk(" error chip_id = %d\n",chip_id);
	    return -ENODEV;
    }
//[ECID:000000] ZTEBSP wanghaifei end 20111125

    if ((ret = (alloc_chrdev_region(&taos_dev_number, 0, TAOS_MAX_NUM_DEVICES, TAOS_DEVICE_NAME))) < 0) {
        printk(KERN_ERR "TAOS: alloc_chrdev_region() failed in taos_init()\n");
        return (ret);
    }
    taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
    if (!taos_datap) {
        printk(KERN_ERR "TAOS: kmalloc for struct taos_data failed in taos_init()\n");
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	ret = -ENOMEM;
	goto malloc1_fail;
        //return -ENOMEM;
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    }
    memset(taos_datap, 0, sizeof(struct taos_data));
    cdev_init(&taos_datap->cdev, &taos_fops);
    taos_datap->cdev.owner = THIS_MODULE;
    if ((ret = (cdev_add(&taos_datap->cdev, taos_dev_number, 1))) < 0) {
        printk(KERN_ERR "TAOS: cdev_add() failed in taos_init()\n");
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	goto cdev_add_fail;
        //return (ret);
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    }

    taos_class = class_create(THIS_MODULE, TAOS_DEVICE_NAME);
    device_create(taos_class, NULL, MKDEV(MAJOR(taos_dev_number), 0), &taos_driver ,"taos");
    wake_lock_init(&taos_datap->taos_wake_lock, WAKE_LOCK_SUSPEND, "taos-wake-lock");


    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        printk(KERN_ERR "TAOS: taos_probe() - i2c smbus byte data functions unsupported\n");
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	ret = -EOPNOTSUPP;
	goto i2c_func_fail;
        //return -EOPNOTSUPP;
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    }
    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
        printk(KERN_ERR "TAOS: taos_probe() - i2c smbus word data functions unsupported\n");
    }
    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BLOCK_DATA)) {
        printk(KERN_ERR "TAOS: taos_probe() - i2c smbus block data functions unsupported\n");
    }
    taos_datap->client = clientp;
    i2c_set_clientdata(clientp, taos_datap);
    INIT_WORK(&(taos_datap->work),taos_work_func);
 //   init_MUTEX(&taos_datap->update_lock); //[ECID:000000] ZTEBSP wanghaifei  20111021, useless

   taos_datap->input_dev = input_allocate_device();//iVIZM
   if (taos_datap->input_dev == NULL) {
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	ret = -EBUSY;
	goto malloc2_fail;
       //return -ENOMEM;
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
   }
   
   taos_datap->input_dev->name = TAOS_INPUT_NAME;
   taos_datap->input_dev->id.bustype = BUS_I2C;

/*[ECID:000000] ZTEBSP wanghaifei start 20111020, should use input_set_abs_params if suppoet EV_ABS, or NULL pointer will be reported*/
   set_bit(EV_ABS,taos_datap->input_dev->evbit);
/*
   input_set_capability(taos_datap->input_dev,EV_ABS,ABS_DISTANCE);
   input_set_capability(taos_datap->input_dev,EV_ABS,ABS_MISC);
   ret = input_register_device(taos_datap->input_dev);
*/
    input_set_abs_params(taos_datap->input_dev, ABS_DISTANCE, 0, 100, 0, 0);
    input_set_abs_params(taos_datap->input_dev, ABS_MISC, 0, 20000, 0, 0);
    if ((ret = input_register_device(taos_datap->input_dev))) {
	goto input_register_fail;
    }
/*[ECID:000000] ZTEBSP wanghaifei start 20111020*/




    strlcpy(clientp->name, TAOS_DEVICE_ID, I2C_NAME_SIZE);
    strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);
/*[ECID:000000] ZTEBSP wanghaifei start 20111020, remove variable useless, add open_num*/
    /*
    taos_datap->valid = 0; 
    */
    taos_datap->open_num = 0; 
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/
    if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL))) {
        printk(KERN_ERR "TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	ret = -ENOMEM;
	goto malloc3_fail;
       //return -ENOMEM;
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    }
    taos_cfgp->calibrate_target = calibrate_target_param;
    taos_cfgp->als_time = als_time_param;
    taos_cfgp->scale_factor = scale_factor_param;
    taos_cfgp->gain_trim = gain_trim_param;
    taos_cfgp->filter_history = filter_history_param;
    taos_cfgp->filter_count = filter_count_param;
    taos_cfgp->gain = gain_param;
    taos_cfgp->als_threshold_hi = als_threshold_hi_param;//iVIZM
    taos_cfgp->als_threshold_lo = als_threshold_lo_param;//iVIZM
    taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
    taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
    taos_cfgp->prox_int_time = prox_int_time_param;
    taos_cfgp->prox_adc_time = prox_adc_time_param;
    taos_cfgp->prox_wait_time = prox_wait_time_param;
    taos_cfgp->prox_intr_filter = prox_intr_filter_param;
    taos_cfgp->prox_config = prox_config_param;
    taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
    taos_cfgp->prox_gain = prox_gain_param;
    sat_als = (256 - taos_cfgp->prox_int_time) << 10;
    sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;

    /*dmobile ::power down for init ,Rambo liu*/
    printk("Rambo::light sensor will pwr down \n");
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x00), 0x00))) < 0) {
        printk(KERN_ERR "TAOS:Rambo, i2c_smbus_write_byte_data failed in power down\n");
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, should release some memory before return */
	goto smbus_fail;
        //return (ret);
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    }

/*[ECID:000000] ZTEBSP wanghaifei start 20111018, modify irq number and add some fail case process*/
/*
     ret =  request_threaded_irq( PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,ALS_PS_GPIO-1),NULL,taos_irq_handler, 
                                  IRQ_TYPE_EDGE_FALLING, "als_irq",taos_datap);
	if (ret) {
		printk("fail to request irq\n");
	}
   disable_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,ALS_PS_GPIO-1));
*/
     ret =  request_threaded_irq( clientp->irq, NULL, taos_irq_handler, 
                                  IRQ_TYPE_EDGE_FALLING, "tms2771_irq", taos_datap);
	if (ret) {
		printk("%s %s fail to request irq\n", __FILE__, __func__);
	goto req_irq_fail;
	}
   disable_irq(clientp->irq);
   return (ret);


req_irq_fail:
smbus_fail:
   kfree(taos_cfgp);
malloc3_fail:
   input_unregister_device(taos_datap->input_dev);
   goto malloc2_fail;
input_register_fail:
   input_free_device(taos_datap->input_dev);
malloc2_fail:
i2c_func_fail:
   device_destroy(taos_class, MKDEV(MAJOR(taos_dev_number), 0));

   class_destroy(taos_class);

   cdev_del(&taos_datap->cdev);
cdev_add_fail:
   kfree(taos_datap);
malloc1_fail:
   unregister_chrdev_region(taos_dev_number, TAOS_MAX_NUM_DEVICES);
   return ret;
/*[ECID:000000] ZTEBSP wanghaifei end 20111018*/

}
//resume  iVIZM
static int taos_resume(struct i2c_client *client) {
    u8 reg_val = 0,reg_cntrl = 0;
    int ret = -1;

/*[ECID:000000] ZTEBSP wanghaifei start 20120319, do nothing if in call*/
   if(taos_datap->working == 2)
   {
        taos_datap->working = 0; 
	disable_irq_wake(client->irq);
        printk("TAOS: in function taos_resume, in call\n");
        return 0;
  }
   printk("TAOS: in function taos_resume, normal resume\n");
/*[ECID:000000] ZTEBSP wanghaifei end 20120319*/

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in taos_resume\n");
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);
    if ( taos_datap->working == 1) {
        taos_datap->working = 0;
        reg_cntrl = reg_val | TAOS_TRITON_CNTL_PWRON;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		taos_datap->working = 1; //[ECID:000000] ZTEBSP wanghaifei 20111021, resume working state if taos_resume failed
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
            return (ret);
        }
    }
/*[ECID:000000] ZTEBSP wanghaifei start 20111018, modify irq number*/
   /*enable_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,ALS_PS_GPIO-1));*/
   enable_irq(client->irq);
/*[ECID:000000] ZTEBSP wanghaifei end 20111018*/
    return ret;
}

//suspend  iVIZM
static int taos_suspend(struct i2c_client *client, pm_message_t mesg) {
    u8 reg_val = 0,reg_cntrl = 0;
    int ret = -1;
/*[ECID:000000] ZTEBSP wanghaifei start 20120319, do nothing if in call*/
   if(in_call_state())
   {
      taos_datap->working = 2;
	enable_irq_wake(client->irq);
        printk("TAOS: in function taos_suspend, in call\n");
        return 0;
  }
   printk("TAOS: in function taos_suspend, normal suspend\n");
   disable_irq(client->irq);
/*[ECID:000000] ZTEBSP wanghaifei end 20120319*/
    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
        printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in taos_resume\n");
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);
    if (reg_val & TAOS_TRITON_CNTL_PWRON) {
        taos_datap->working = 1;
        reg_cntrl = reg_val & (~TAOS_TRITON_CNTL_PWRON);
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos_suspend\n");
            return (ret);
        }
    }
    return ret;
}
// client remove
static int __devexit taos_remove(struct i2c_client *client) {
    int ret = 0;

    return (ret);
}
// open
static int taos_open(struct inode *inode, struct file *file) {
//    struct taos_data *taos_datap; //[ECID:000000] ZTEBSP wanghaifei start 20111018, modify irq number
    int ret = 0;

/*[ECID:000000] ZTEBSP wanghaifei start 20111018, modify irq number*/
/*
    device_released = 0;
    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
    printk("TAOS: device id %s\n",taos_datap->taos_name);
    if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) {
        printk(KERN_ERR "TAOS: device name incorrect during taos_open(), get %s\n", taos_datap->taos_name);
        ret = -ENODEV;
    }
    memset(readdata, 0, sizeof(struct ReadData)*2);//iVIZM
  //  enable_irq(PM8058_GPIO_TO_IRQS(ALS_PS_GPIO-1));//iVIZM
     enable_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE,ALS_PS_GPIO-1));
  */
    taos_datap->open_num += 1;
    printk("TAOS:  open number %d\n", taos_datap->open_num);
    if (taos_datap->open_num == 1) {
	    enable_irq(taos_datap->client->irq);
    }
/*[ECID:000000] ZTEBSP wanghaifei end 20111018*/
    return (ret);
}

// release
static int taos_release(struct inode *inode, struct file *file) {
//    struct taos_data *taos_datap; //ECID:000000] ZTEBSP wanghaifei start 20111020, remove variable useless
    int ret = 0;

/*[ECID:000000] ZTEBSP wanghaifei start 20111020, remove variable useless*/
//    device_released = 1;
    prox_on = 0;
  taos_datap->open_num -= 1;
    printk("TAOS:  close number %d\n", taos_datap->open_num);
  if (taos_datap->open_num <= 0) {
	  taos_datap->open_num = 0;
	  disable_irq(taos_datap->client->irq);
  }
/*
    prox_history_hi = 0;
    prox_history_lo = 0;
    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
    if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) {
        printk(KERN_ERR "TAOS: device name incorrect during taos_release(), get %s\n", taos_datap->taos_name);
        ret = -ENODEV;
    }
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/
    return (ret);
}

/*[ECID:000000] ZTEBSP wanghaifei start 20111020, remove some function useless*/
/*
// read
static int taos_read(struct file *file, char *buf, size_t count, loff_t *ppos) {
    unsigned long flags;
    int realmax;
    int err;
  printk("taos: taos_read\n");
    if((!ReadEnable) && (file->f_flags & O_NONBLOCK))
        return -EAGAIN;
    local_save_flags(flags);
    local_irq_disable();

    realmax = 0;
    if (down_interruptible(&taos_datap->update_lock))
        return -ERESTARTSYS;
    if (ReadEnable > 0) {
        if (sizeof(struct ReadData)*2 < count)
            realmax = sizeof(struct ReadData)*2;
        else
            realmax = count;
        err = copy_to_user(buf, readdata, realmax);
        if (err) 
            return -EAGAIN;
        ReadEnable = 0;
    }
    up(&taos_datap->update_lock);
    memset(readdata, 0, sizeof(struct ReadData)*2);
    local_irq_restore(flags);
    return realmax;
}

// write
static int taos_write(struct file *file, const char *buf, size_t count, loff_t *ppos) {
    struct taos_data *taos_datap;
    u8 i = 0, xfrd = 0, reg = 0;
    u8 my_buf[TAOS_MAX_DEVICE_REGS];
    int ret = 0;

    if ((*ppos < 0) || (*ppos >= TAOS_MAX_DEVICE_REGS) || ((*ppos + count) > TAOS_MAX_DEVICE_REGS)) {
        printk(KERN_ERR "TAOS: reg limit check failed in taos_write()\n");
        return -EINVAL;
    }
    reg = (u8)*ppos;
    if ((ret =  copy_from_user(my_buf, buf, count))) {
        printk(KERN_ERR "TAOS: copy_to_user failed in taos_write()\n");
        return -ENODATA;
    }
    taos_datap = container_of(file->f_dentry->d_inode->i_cdev, struct taos_data, cdev);
    while (xfrd < count) {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | reg), my_buf[i++]))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos_write()\n");
            return (ret);
        }
        reg++;
        xfrd++;
     }
     return ((int)xfrd);
}

// llseek
static loff_t taos_llseek(struct file *file, loff_t offset, int orig) {
    int ret = 0;
    loff_t new_pos = 0;

    if ((offset >= TAOS_MAX_DEVICE_REGS) || (orig < 0) || (orig > 1)) {
        printk(KERN_ERR "TAOS: offset param limit or origin limit check failed in taos_llseek()\n");
        return -EINVAL;
    }
    switch (orig) {
    case 0:
        new_pos = offset;
        break;
    case 1:
        new_pos = file->f_pos + offset;
        break;
    default:
        return -EINVAL;
        break;
    }
    if ((new_pos < 0) || (new_pos >= TAOS_MAX_DEVICE_REGS) || (ret < 0)) {
        printk(KERN_ERR "TAOS: new offset limit or origin limit check failed in taos_llseek()\n");
        return -EINVAL;
    }
    file->f_pos = new_pos;
    return new_pos;
}
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/

static int taos_sensors_als_on(void) {
    int  ret = 0, i = 0;
    u8 itime = 0, reg_val = 0, reg_cntrl = 0;
    //int lux_val = 0, ret = 0, i = 0, tmp = 0;

            for (i = 0; i < TAOS_FILTER_DEPTH; i++)
                lux_history[i] = -ENODATA;
            //taos_als_threshold_set();//iVIZM
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_on\n");
                return (ret);
            }
            itime = (((taos_cfgp->als_time/50) * 18) - 1);
            itime = (~itime);
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), taos_cfgp->prox_intr_filter))) < 0) {//golden
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_on\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            reg_val = reg_val & 0xFC;
            reg_val = reg_val | (taos_cfgp->gain & 0x03);
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }
            reg_cntrl = (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ALS_INT_ENBL);
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
            }
            taos_als_threshold_set();//iVIZM
			return ret;
}	


// ioctls
//[ECID:000000] ZTEBSP wanghaifei start 20111018, remove ioctl since 2.6.36
//static int taos_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg) {
static long taos_unlocked_ioctl(struct file *fp, unsigned int cmd, unsigned long arg) {
//[ECID:000000] ZTEBSP wanghaifei end 20111018
    struct taos_data *taos_datap;
    int prox_sum = 0, prox_mean = 0, prox_max = 0;
    int lux_val = 0, ret = 0, i = 0, tmp = 0;
    u16 gain_trim_val = 0;
    u8 reg_val = 0, reg_cntrl = 0;
/*[ECID:000000] ZTEBSP wanghaifei start 20111021, remove some variable  useless */
	/*
    int ret_check=0;
    int ret_m=0;
    u8 reg_val_temp=0;
     uint taos_config_set = TAOS_IOCTL_CONFIG_SET;
   u8 id_value;
    printk("TAOS: taos_ioctl cmd is:%d, config_set is:%d \n", cmd, taos_config_set);

    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
   */
    taos_datap = container_of(fp->f_dentry->d_inode->i_cdev, struct taos_data, cdev);
/*[ECID:000000] ZTEBSP wanghaifei end 20111021*/
    switch (cmd) {
	    case TAOS_IOCTL_SENSOR_CHECK:
		  //  u8 reg_val_temp=0; //ECID:000000] ZTEBSP wanghaifei 20111021, reg_val instead of reg_val_temp
		    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
			    printk(KERN_ERR "TAOS: TAOS_IOCTL_SENSOR_CHECK failed\n");
			    return (ret);
		    }
		    reg_val = i2c_smbus_read_byte(taos_datap->client);
		    if ((reg_val & 0xFF) == 0xF)
			    return -ENODATA;

		    break;

	    case TAOS_IOCTL_SENSOR_CONFIG:
		    ret = copy_from_user(taos_cfgp, (struct taos_cfg *)arg, sizeof(struct taos_cfg));
		    if (ret) {
			    printk(KERN_ERR "TAOS: copy_from_user failed in ioctl config_set\n");
			    return -ENODATA;
		    }

		    break;

/*[ECID:000000] ZTEBSP wanghaifei start 20111021, remove case useless */
	/*
	    case TAOS_IOCTL_SENSOR_READ_DATA:
		    taos_prox_threshold_set();
		    break;
	*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111021 */

	    case TAOS_IOCTL_SENSOR_ON:
		    ret=0;
#if 1
		    /*Register init and turn off */
        for (i = 0; i < TAOS_FILTER_DEPTH; i++){
            /*Rambo ??*/
            lux_history[i] = -ENODATA;
        }
#endif

        /*ALS interrupt clear*/
        if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_on\n");
            return (ret);
        }

        /*Register setting*/
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), taos_cfgp->prox_int_time))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->prox_wait_time))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), taos_cfgp->prox_intr_filter))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG), taos_cfgp->prox_config))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }
        /*gain*/
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }

        /*turn on*/
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CNTRL), 0xF))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
            return (ret);
        }
        break;


        case TAOS_IOCTL_SENSOR_OFF:
            ret=0;

            /*turn off*/
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x00), 0x00))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_off\n");
                return (ret);
            }
             break;

        case TAOS_IOCTL_ALS_ON:
		//[ECID:000000]ZTEBSP DangXiao 20120116 start, for Taos bug
		/////////////Read Process 1 /////////		
		reg_val = i2c_smbus_read_byte_data(taos_datap->client, TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
		//printk("TAOS: reg_ctl als on 1 = 0x%x\n", reg_val);
		//////////////////////////////////
		
	     ////////Read  Process 2/////////	
	     /*	
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_calibrate\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
			printk("TAOS: reg_ctl als on 2= 0x%x\n", reg_val);
	     */
	     //////////////////
		 //[ECID:000000]ZTEBSP DangXiao 20120116 end
	    if ((reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL) == 0x0) {
		    taos_sensors_als_on();
	    }	

	    ALS_ON = 1;
	    return (ret);
            break;
        case TAOS_IOCTL_ALS_OFF:
            for (i = 0; i < TAOS_FILTER_DEPTH; i++)
                lux_history[i] = -ENODATA;
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_calibrate\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            if ((reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL) == 0x0) {
               if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                   printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
		   return (ret);
	       }
	       cancel_work_sync(&taos_datap->work);//golden
            }
	    ALS_ON = 0;
            return (ret);
            break;
        case TAOS_IOCTL_ALS_DATA:
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
                return -ENODATA;
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
                return -ENODATA;
            if ((lux_val = taos_get_lux()) < 0)
                printk(KERN_ERR "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val);
            lux_val = taos_lux_filter(lux_val);
            return (lux_val);
            break;
        case TAOS_IOCTL_ALS_CALIBRATE:
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_calibrate\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            if ((reg_val & 0x07) != 0x07)
                return -ENODATA;
            if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte failed in ioctl als_calibrate\n");
                return (ret);
            }
            reg_val = i2c_smbus_read_byte(taos_datap->client);
            if ((reg_val & 0x01) != 0x01)
                return -ENODATA;
            if ((lux_val = taos_get_lux()) < 0) {
                printk(KERN_ERR "TAOS: call to lux_val() returned error %d in ioctl als_data\n", lux_val);
                return (lux_val);
            }
            gain_trim_val = (u16)(((taos_cfgp->calibrate_target) * 512)/lux_val);
            taos_cfgp->gain_trim = (int)gain_trim_val;
            return ((int)gain_trim_val);
            break;
        case TAOS_IOCTL_CONFIG_GET:
            ret = copy_to_user((struct taos_cfg *)arg, taos_cfgp, sizeof(struct taos_cfg));
            if (ret) {
                printk(KERN_ERR "TAOS: copy_to_user failed in ioctl config_get\n");
                return -ENODATA;
            }
            return (ret);
            break;
        case TAOS_IOCTL_CONFIG_SET:
            printk("^^^^^^^^^ TAOS INCTL CONFIG SET  ^^^^^^^\n");
            ret = copy_from_user(taos_cfgp, (struct taos_cfg *)arg, sizeof(struct taos_cfg));
            if (ret) {
                printk(KERN_ERR "TAOS: copy_from_user failed in ioctl config_set\n");
                return -ENODATA;
            }
            if(taos_cfgp->als_time < 50)
                taos_cfgp->als_time = 50;
            if(taos_cfgp->als_time > 650)
                taos_cfgp->als_time = 650;
            tmp = (taos_cfgp->als_time + 25)/50;
            taos_cfgp->als_time = tmp*50;
            sat_als = (256 - taos_cfgp->prox_int_time) << 10;
            sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;
            break;
        case TAOS_IOCTL_PROX_ON:
            printk("^^^^^^^^^ TAOS IOCTL PROX ON  ^^^^^^^\n");
            prox_on = 1;

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x01), taos_cfgp->prox_int_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x02), taos_cfgp->prox_adc_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x03), taos_cfgp->prox_wait_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0C), taos_cfgp->prox_intr_filter))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0D), taos_cfgp->prox_config))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0E), taos_cfgp->prox_pulse_cnt))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0F), taos_cfgp->prox_gain))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_PROX_INT_ENBL | 
				                                      TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL  ;
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            taos_prox_threshold_set();//iVIZM
            break;
        case TAOS_IOCTL_PROX_OFF:
            printk("^^^^^^^^^ TAOS IOCTL PROX OFF  ^^^^^^^\n");
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
                return (ret);
            }
			if (ALS_ON == 1) {
                taos_sensors_als_on();
			} else {
				cancel_work_sync(&taos_datap->work);//golden
			}
            prox_on = 0;
            break;
        case TAOS_IOCTL_PROX_DATA:
            //Rambo
#if 1
            if ((ret = taos_prox_poll(prox_cur_infop)) < 0) {
                printk(KERN_ERR "TAOS: call to prox_poll failed in taos_prox_poll_timer_func()\n");
                return ret;
            }
#endif
            ret = copy_to_user((struct taos_prox_info *)arg, prox_cur_infop, sizeof(struct taos_prox_info));
            if (ret) {
                printk(KERN_ERR "TAOS: copy_to_user failed in ioctl prox_data\n");
                return -ENODATA;
            }
            return (ret);
            break;
        case TAOS_IOCTL_PROX_EVENT:
            if ((ret = taos_prox_poll(prox_cur_infop)) < 0) {
                printk(KERN_ERR "TAOS: call to prox_poll failed in taos_prox_poll_timer_func()\n");
                return ret;
            }

            return (prox_cur_infop->prox_event);
            break;

/*
	  case TAOS_IOCTL_READ_ID:

	    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + TAOS_TRITON_CHIPID))))) < 0) {
		    printk(KERN_ERR "TAOS: i2c_smbus_write_byte() to control reg failed in taos_ioctl()\n");
		    return -ENODATA;
	    }
	    id_value = i2c_smbus_read_byte(taos_datap->client);
	    printk("taos:id value is:%d\n",id_value);
	    return id_value;
	    break;
*/
  	
	
        case TAOS_IOCTL_PROX_CALIBRATE:
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x01), taos_cfgp->prox_int_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x02), taos_cfgp->prox_adc_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x03), taos_cfgp->prox_wait_time))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0D), taos_cfgp->prox_config))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0E), taos_cfgp->prox_pulse_cnt))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x0F), taos_cfgp->prox_gain))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }
            reg_cntrl = reg_val | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
            }

            prox_sum = 0;
            prox_max = 0;
            for (i = 0; i < 20; i++) {
                if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
                    printk(KERN_ERR "TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
                    return (ret);
                }
                prox_sum += prox_cal_info[i].prox_data;
                if (prox_cal_info[i].prox_data > prox_max)
                    prox_max = prox_cal_info[i].prox_data;
                mdelay(100);
            }
            prox_mean = prox_sum/20;
            taos_cfgp->prox_threshold_hi = ((((prox_max - prox_mean) * 200) + 50)/100) + prox_mean;
            taos_cfgp->prox_threshold_lo = ((((prox_max - prox_mean) * 170) + 50)/100) + prox_mean;
//[ECID:000000] ZTEBSP wanghaifei start 20120229, for calibration failed
            printk("TAOS: prox_threshold_lo = 0x%x, prox_threashold_hi = 0x%x\n", taos_cfgp->prox_threshold_lo, taos_cfgp->prox_threshold_hi);
            if (taos_cfgp->prox_threshold_hi >= 0x350) {
                printk("TAOS: use default threshold\n");
                taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
                taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
            }    
//[ECID:000000] ZTEBSP wanghaifei end 20120229, for calibration failed
            for (i = 0; i < sizeof(taos_triton_reg_init); i++){
                if(i !=11){
                    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i]))) < 0) {
                        printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                        return (ret);
                    }
                 }
             }

            break;
        default:
            return -EINVAL;
            break;
    }
    return (ret);
}
// read/calculate lux value
static int taos_get_lux(void) {
    u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
    u32 lux = 0;
    u32 ratio = 0;
    u8 dev_gain = 0;
    u16 Tint = 0;
    struct lux_data *p;
    int ret = 0;
    u8 chdata[4];
    int tmp = 0, i = 0;

    for (i = 0; i < 4; i++) {
        if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))))) < 0) {
            printk(KERN_ERR "TAOS: i2c_smbus_write_byte() to chan0/1/lo/hi reg failed in taos_get_lux()\n");
            return (ret);
        }
        chdata[i] = i2c_smbus_read_byte(taos_datap->client);
    }
    printk("ch0=%d\n",chdata[0]+chdata[1]*256);
    printk("ch1=%d\n",chdata[2]+chdata[3]*256);

    tmp = (taos_cfgp->als_time + 25)/50;            //if atime =100  tmp = (atime+25)/50=2.5   tine = 2.7*(256-atime)=  412.5
    TritonTime.numerator = 1;
    TritonTime.denominator = tmp;

    tmp = 300 * taos_cfgp->als_time;               //tmp = 300*atime  400
    if(tmp > 65535)
        tmp = 65535;
    TritonTime.saturation = tmp;
    raw_clear = chdata[1];
    raw_clear <<= 8;
    raw_clear |= chdata[0];
    raw_ir    = chdata[3];
    raw_ir    <<= 8;
    raw_ir    |= chdata[2];

    raw_clear *= (taos_cfgp->scale_factor * 11);   
    raw_ir *= (taos_cfgp->scale_factor * 3);

    if(raw_ir > raw_clear) {
        raw_lux = raw_ir;
        raw_ir = raw_clear;
        raw_clear = raw_lux;
    }
    dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
    if(raw_clear >= lux_timep->saturation)
        return(TAOS_MAX_LUX);
    if(raw_ir >= lux_timep->saturation)
        return(TAOS_MAX_LUX);
    if(raw_clear == 0)
        return(0);
    if(dev_gain == 0 || dev_gain > 127) {
        printk(KERN_ERR "TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
        return -1;
    }
    if(lux_timep->denominator == 0) {
        printk(KERN_ERR "TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
        return -1;
    }
    ratio = (raw_ir<<15)/raw_clear;
    for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);
    if(!p->ratio) {//iVIZM
    return 10000;
        if(lux_history[0] < 0)
            return 0;
        else
            return lux_history[0];
    }
    Tint = taos_cfgp->als_time;
    raw_clear = ((raw_clear*400 + (dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    raw_ir = ((raw_ir*400 +(dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
    lux = (lux + 32000)/64000;
    if(lux > TAOS_MAX_LUX) {
        lux = TAOS_MAX_LUX;
    }
    //return(lux)*taos_cfgp->filter_count;
    return(lux);
}

static int taos_lux_filter(int lux)
{
    static u8 middle[] = {1,0,2,0,0,2,0,1};
    int index;

    lux_history[2] = lux_history[1];
    lux_history[1] = lux_history[0];
    lux_history[0] = lux;

    if(lux_history[2] < 0) { //iVIZM
        if(lux_history[1] > 0)
            return lux_history[1];       
        else 
            return lux_history[0];
    }
    index = 0;
    if( lux_history[0] > lux_history[1] ) 
        index += 4;
    if( lux_history[1] > lux_history[2] ) 
        index += 2;
    if( lux_history[0] > lux_history[2] )
        index++;
    return(lux_history[middle[index]]);
}

// verify device

// proximity poll
static int taos_prox_poll(struct taos_prox_info *prxp) {
    int i = 0, ret = 0; 
    u8 chdata[6];
    for (i = 0; i < 6; i++) {
        chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_AUTO | (TAOS_TRITON_ALS_CHAN0LO + i))));
    }
    prxp->prox_clear = chdata[1];
    prxp->prox_clear <<= 8;
    prxp->prox_clear |= chdata[0];
    if (prxp->prox_clear > ((sat_als*80)/100))
        return -ENODATA;
    prxp->prox_data = chdata[5];
    prxp->prox_data <<= 8;
    prxp->prox_data |= chdata[4];


    return (ret);
}

/*[ECID:000000] ZTEBSP wanghaifei start 20111020, remove function useless*/
// prox poll timer function
/*
static void taos_prox_poll_timer_func(unsigned long param) {
    int ret = 0;

    if (!device_released) {
     if ((ret = taos_prox_poll(prox_cur_infop)) < 0) {
	    printk(KERN_ERR "TAOS: call to prox_poll failed in taos_prox_poll_timer_func()\n");
            return;
        }
        taos_prox_poll_timer_start();
    }
    return;
}

// start prox poll timer
static void taos_prox_poll_timer_start(void) {
    init_timer(&prox_poll_timer);
    prox_poll_timer.expires = jiffies + (HZ/10);
    prox_poll_timer.function = taos_prox_poll_timer_func;
    add_timer(&prox_poll_timer);
    return;
}
*/
/*[ECID:000000] ZTEBSP wanghaifei end 20111020*/


MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

late_initcall(taos_init);
module_exit(taos_exit);

