#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
//#include <cust_gpio_usage.h>
//#include <asm-generic/gpio.h>
#include <linux/gpio.h>
#include "cci_hw_id.h"

/*
  bit 0 -- HW ID 1 (GPIO56)
  bit 1 -- HW ID 2 (GPIO57)
  bit 2 -- HW ID 3 (GPIO58)

  bit 3 -- Project ID 1 (GPIO90)
  bit 4 -- Project ID 2 (GPIO89)
  bit 5 -- Project ID 3 (GPIO80)
  to be con.
*/
int cci_hw_id = 0;
char* cci_phase_name;

char cci_board_type_str[][20] = 
{
"EVT1 board",      // 0 0 0
"DVT1-1 board",  // 0 0 1
"DVT1-2 board",  // 0 1 0
"DVT2 board",     // 0 1 1
"DVT3 board",     // 1 0 0
"TP1 board",        // 1 0 1
"PVT board",       // 1 1 0
""
};

char cci_project_str[][20] = 
{
"VY36", // 0 0 0
"VY37", // 0 0 1	
"VY38", // 0 1 0
"Unknow",        // 0 1 1
"VY39", // 1 0 0
"VY40", // 1 0 1
""
};

char cci_rf_str[][20] = 
{
"Gina",          // 0 0
"Rex",           // 0 1	
"Gina APAC", // 0 1
""                 // 1 1
};

int get_cci_hw_id(void)
{
	return cci_hw_id;
}
EXPORT_SYMBOL(get_cci_hw_id);

char* get_cci_phase_name(void)
{
	return cci_phase_name;
}
EXPORT_SYMBOL(get_cci_phase_name);

int board_type_with_hw_id(void)
{
    //return ( ( (cci_hw_id>>4) & 0x07 ) + 1);
    return (cci_hw_id & 0x07);
}
EXPORT_SYMBOL(board_type_with_hw_id);

int project_id_with_hw_id(void)
{
    return ( ((cci_hw_id>>3) & 0x07));
}
EXPORT_SYMBOL(project_id_with_hw_id);

char* get_cci_hw_id_name(int value)
{
    char* board_name;

    if (value >= 7 || value < 0)
    {
        printk("[HWID] EROOR: Not support HW ID numebr %d \n", value);
        board_name = cci_board_type_str[7];
    }
    else
    {
        board_name = cci_board_type_str[value];
        printk("[HWID] board_name=%s HW ID numebr %d \n", board_name, value);
    }

    return board_name;
}

const char* get_cci_project_id_name(int value)
{
    char* board_name;

    if (value >= 6 || value < 0)
    {
        printk("[HWID] EROOR: Not support Project ID numebr %d \n", value);
        board_name = cci_project_str[5];
    }
    else
    {
        board_name = cci_project_str[value];
        printk("[HWID] project_name=%s project ID numebr %d \n", board_name, value);
    }

    return board_name;
}

const char* get_cci_rf_id_name(int value)
{
    char* board_name;

    if (value >= 3 || value < 0)
    {
        printk("[HWID] EROOR: Not support RF ID numebr %d \n", value);
        board_name = cci_rf_str[3];
    }
    else
    {
        board_name = cci_rf_str[value];
        printk("[HWID] rf_name=%s RF ID numebr %d \n", board_name, value);
    }

    return board_name;
}

#define HWID_1 56
#define HWID_2 57
#define HWID_3 58
#define HWID_4 59
#define HWID_5 60  
#define HWID_8 90
#define HWID_9 89
#define HWID_10 80
#define GPIO_CEI_CTP_ID 23
#define GPIO_LCM_ID 54
void cci_det_hw_id(void)
{
	#if 1
	int tmp_id = 0;
	int board_type =0;
	int project_id = 0;
//	int rf_id = 0;
	//char* project_name;

	//Detect HW ID

	//Phase ID GPIO56/57/58-->Begin
	printk("[HWID] cci_det_hw_id enter\n");
	mt_set_gpio_mode(HWID_1, 0); 
	mt_set_gpio_dir(HWID_1, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(HWID_1, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_1) == 1) tmp_id|= SET_MODEL_TYPE_3;

	mt_set_gpio_mode(HWID_2, 0); 
	mt_set_gpio_dir(HWID_2, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(HWID_2, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_2) == 1) tmp_id|= SET_MODEL_TYPE_2;

	mt_set_gpio_mode(HWID_3, 0); 
	mt_set_gpio_dir(HWID_3, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(HWID_3, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_3) == 1) tmp_id|= SET_MODEL_TYPE_1;
	//Phase ID GPIO56/57/58-->End
	

	cci_hw_id = tmp_id;

	board_type = board_type_with_hw_id();
	cci_phase_name = get_cci_hw_id_name(board_type);

	//Project ID--B
	//Project ID GPIO90/89/80-->Begin
	mt_set_gpio_mode(HWID_8, 0); 
	mt_set_gpio_dir(HWID_8, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(HWID_8, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_8) == 1) tmp_id|= SET_MODEL_TYPE_6;

	mt_set_gpio_mode(HWID_9, 0); 
	mt_set_gpio_dir(HWID_9, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
       mt_set_gpio_pull_enable(HWID_9, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_9) == 1) tmp_id|= SET_MODEL_TYPE_5;

	mt_set_gpio_mode(HWID_10, 0); // 
	mt_set_gpio_dir(HWID_10, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(HWID_10, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(HWID_10) == 1) tmp_id|= SET_MODEL_TYPE_4;
	//Project ID GPIO90/89/80-->end

	cci_hw_id = tmp_id;
	project_id = project_id_with_hw_id();
	//Project ID--E

	//printk("//board_get_hw_id [%x], %s\n", cci_hw_id, 
   	//cci_board_type_str[board_type - 1]);
	printk("[HWID] board_get_hw_id [%x], %s, %s\n", cci_hw_id, get_cci_phase_name(), get_cci_project_id_name(project_id));

#if 0 //RFID remove
       //RF ID -- B
	//Project ID GPIO59/60
	mt_set_gpio_mode(GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN, GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN_M_GPIO); 
	mt_set_gpio_dir(GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN) == 1) rf_id|= SET_MODEL_TYPE_2;

	mt_set_gpio_mode(GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN, GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN_M_GPIO); 
	mt_set_gpio_dir(GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN) == 1) rf_id|= SET_MODEL_TYPE_1;

	printk("[HWID] RF ID [%x], %s\n", rf_id, get_cci_rf_id_name(rf_id));

       //RF ID -- E
 #endif //RFID remove
 
#else
	int tmp_id = 0;
	int board_type =0;
	int project_id = 0;
	int rf_id = 0;
	
  //gpio_request(56, "HWID_1");
	//tmp_id = __gpio_get_value(56);
	//gpio_free(56);
	//printk("[HWID] cci_det_hw_id enter tmp_id=%d\n", tmp_id);

	//Phase ID GPIO56/57/58-->Begin
	printk("[HWID] cci_det_hw_id enter\n");

  gpio_request(HWID_1, "HWID_1");
	gpio_direction_input(HWID_1);
	if (__gpio_get_value(HWID_1) == 1) tmp_id|= SET_MODEL_TYPE_3;
	gpio_free(HWID_1);

  gpio_request(HWID_2, "HWID_2");
	gpio_direction_input(HWID_2);
	if (__gpio_get_value(HWID_2) == 1) tmp_id|= SET_MODEL_TYPE_2;
	gpio_free(HWID_2);

  gpio_request(HWID_3, "HWID_3");
	gpio_direction_input(HWID_3);
	if (__gpio_get_value(HWID_3) == 1) tmp_id|= SET_MODEL_TYPE_1;
	gpio_free(HWID_3);

	//Phase ID GPIO56/57/58-->End

	cci_hw_id = tmp_id;

	board_type = board_type_with_hw_id();
	cci_phase_name = get_cci_hw_id_name(board_type);

	//Project ID--B
	//Project ID GPIO90/89/80-->Begin
  gpio_request(HWID_8, "HWID_8");
	gpio_direction_input(HWID_8);
	if (__gpio_get_value(HWID_8) == 1) tmp_id|= SET_MODEL_TYPE_6;
	gpio_free(HWID_8);

  gpio_request(HWID_9, "HWID_9");
	gpio_direction_input(HWID_9);
	if (__gpio_get_value(HWID_9) == 1) tmp_id|= SET_MODEL_TYPE_5;
	gpio_free(HWID_9);

  gpio_request(HWID_10, "HWID_10");
	gpio_direction_input(HWID_10);
	if (__gpio_get_value(HWID_10) == 1) tmp_id|= SET_MODEL_TYPE_4;
	gpio_free(HWID_10);
	//Project ID GPIO90/89/80-->end

	cci_hw_id = tmp_id;
	project_id = project_id_with_hw_id();
	//Project ID--E

	//printk("//board_get_hw_id [%x], %s\n", cci_hw_id, 
   	//cci_board_type_str[board_type - 1]);
	printk("[HWID] board_get_hw_id [%x], %s, %s\n", cci_hw_id, get_cci_phase_name(), get_cci_project_id_name(project_id));

  //RF ID -- B
	//Project ID GPIO59/60
  gpio_request(HWID_4, "HWID_4");
	gpio_direction_input(HWID_4);
	if (__gpio_get_value(HWID_4) == 1) tmp_id|= SET_MODEL_TYPE_2;
	gpio_free(HWID_4);

  gpio_request(HWID_5, "HWID_5");
	gpio_direction_input(HWID_5);
	if (__gpio_get_value(HWID_5) == 1) tmp_id|= SET_MODEL_TYPE_1;
	gpio_free(HWID_5);

	printk("[HWID] RF ID [%x], %s\n", rf_id, get_cci_rf_id_name(rf_id));

       //RF ID -- E
#endif //0
	return ;
	
	//Detect HW ID
}

// Luke
#define FTM_PIN_GPIO 15
int get_ftm_pin(void)
{
	int ftm_pin = 0;
#if 1
        mt_set_gpio_mode(FTM_PIN_GPIO, 0);//GPIO mode
        mt_set_gpio_dir(FTM_PIN_GPIO, GPIO_DIR_OUT);
        mt_set_gpio_out(FTM_PIN_GPIO, GPIO_OUT_ONE);

	mt_set_gpio_mode(FTM_PIN_GPIO, 0);
	mt_set_gpio_dir(FTM_PIN_GPIO, GPIO_DIR_IN); // GPIO_DIR_IN or GPIO_DIR_OUT
	mt_set_gpio_pull_enable(FTM_PIN_GPIO, GPIO_PULL_DISABLE); // GPIO_PULL_DISABLE or GPIO_PULL_ENABLE
	if (mt_get_gpio_in(FTM_PIN_GPIO) == 0)    ftm_pin= 1;
	printk("[FTM] ftm_pin, %d\n", ftm_pin );
#else
    printk("[FTM] get_ftm_pin E\n" );

    gpio_request(FTM_PIN_GPIO, "FTM_PIN_GPIO");
    gpio_direction_output(FTM_PIN_GPIO, 1);

    printk("[FTM] get_ftm_pin\n" );
    gpio_direction_input(FTM_PIN_GPIO);
    if (__gpio_get_value(FTM_PIN_GPIO) == 0)    ftm_pin= 1;
    gpio_free(FTM_PIN_GPIO);
    printk("[FTM] ftm_pin, %d\n", ftm_pin );
#endif

	return ftm_pin;
	//FTM ID GPIO15-->End
}

// Mark add proc file for LCM ID
int get_tp_id(void)
{
	int tpid_value = 2;
	//We use TP ID to identify main or second source LCM module
	mt_set_gpio_mode( GPIO_CEI_CTP_ID, 0 );
           mt_set_gpio_dir( GPIO_CEI_CTP_ID, 0 );
	if(mt_get_gpio_in(GPIO_CEI_CTP_ID)==0) {
		tpid_value = 0;
	}
	else if(mt_get_gpio_in(GPIO_CEI_CTP_ID)==1) {
		tpid_value = 1;
	}
	
	printk("[TP ID] tpid_pin= %d\n", tpid_value );
	return tpid_value;
}

int get_lcm_id(void)
{
	int lcmid_value = 2;
	//Get LCM ID pin value
	mt_set_gpio_mode( GPIO_LCM_ID, 0 );
           mt_set_gpio_dir( GPIO_LCM_ID, 0 );

	if(mt_get_gpio_in(GPIO_LCM_ID)==0) {
		lcmid_value = 0;
	}
	else if(mt_get_gpio_in(GPIO_LCM_ID)==1) {
		lcmid_value = 1;
	}
	
	printk("[LCM ID] lcmid_pin, %d\n", lcmid_value );
	return lcmid_value;
}

static int board_type_proc_show(struct seq_file *m, void *v)
{

	int board_type;
	char* name;

	board_type = board_type_with_hw_id();
	name = get_cci_hw_id_name(board_type);
	//seq_printf(m, "%s\n",
		//cci_board_type_str[board_type - 1]);
	seq_printf(m, "%s\n",name);

	return 0;
}

static int cci_hwid_info_proc_show(struct seq_file *m, void *v)
{
	int hwid,projid;
	//hwid = get_cci_hw_id();
	hwid = board_type_with_hw_id();
	projid = project_id_with_hw_id();

	seq_printf(m, "%s=%d %s=%d %s=%d\n", "hwid", hwid, "phaseid", board_type_with_hw_id(), "projectid", projid);

	return 0;
}

// Luke
static int cci_ftm_pin_proc_show(struct seq_file *m, void *v)
{
	int ftm_pin;
	ftm_pin = get_ftm_pin();

	seq_printf(m, "%d\n", ftm_pin);

	return 0;
}

// Mark add proc file for LCM ID
static int cci_lcmid_info_proc_show(struct seq_file *m, void *v)
{
	int lcmid, tpid;
    int boardhwid;
	lcmid = get_lcm_id();
	//We use TP ID to identify main or second source LCM module
	tpid = get_tp_id()+1;
	
	seq_printf(m, "%s=%d \n", "lcm_id_pin", lcmid);
	boardhwid = board_type_with_hw_id();
	if(boardhwid == 6)
	{
		if( lcmid == 1 )
			seq_printf(m, "It's LCM 1st source");
		else if( lcmid == 0 )
			seq_printf(m, "It's LCM 2nd source");
	}
	else
	{
		seq_printf(m, "%s=%d \n", "lcm_source", tpid);
	}
	return 0;
}

static int board_type_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, board_type_proc_show, NULL);
}

static int cci_hwid_info_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, cci_hwid_info_proc_show, NULL);
}

// Luke
static int cci_ftm_pin_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, cci_ftm_pin_proc_show, NULL);
}

//Mark add proc file for LCM ID
static int cci_lcmid_info_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, cci_lcmid_info_proc_show, NULL);
}

static const struct file_operations board_type_proc_fops = {
	.open		= board_type_open_proc,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cci_hwid_info_proc_fops = {
	.open		= cci_hwid_info_open_proc,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

// Luke
static const struct file_operations cci_ftm_pin_proc_fops = {
	.open		= cci_ftm_pin_open_proc,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//Mark add proc file for LCM ID
static const struct file_operations cci_lcmid_info_proc_fops = {
	.open		= cci_lcmid_info_open_proc,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init cci_hw_id_init(void)
{
	

	int err = 0;
	printk("[HWID] cci_hwid_init enter\n");
	cci_det_hw_id();

	proc_create("cci_hw_board_type",0,NULL,&board_type_proc_fops);
	proc_create("cci_hwid_info",0,NULL,&cci_hwid_info_proc_fops);
        // Luke
	proc_create("cci_ftm_pin",0,NULL,&cci_ftm_pin_proc_fops);
	// Mark add proc file for LCM ID
	proc_create("cci_lcm_id",0,NULL,&cci_lcmid_info_proc_fops);
	
	return err;
}

static void __exit cci_hw_id_exit(void)
{
	printk("[HWID] cci_hwid_exit enter\n");
}

module_init(cci_hw_id_init);
module_exit(cci_hw_id_exit);

MODULE_DESCRIPTION("cci hardware ID driver");
MODULE_LICENSE("GPL");
