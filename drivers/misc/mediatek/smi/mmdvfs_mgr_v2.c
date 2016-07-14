#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/mtk_gpu_utility.h>

#include <aee.h>
#include <mt_smi.h>

#ifdef SMI_EV
#define MMDVFS_E1
#endif /* SMI_EV */

#include <mt_vcorefs_manager.h>
#include <mach/mt_freqhopping.h>
#include "mmdvfs_mgr.h"

#undef pr_fmt
#define pr_fmt(fmt) "[" MMDVFS_LOG_TAG "]" fmt

/* MMDVFS SWITCH. NO MMDVFS for 6595 */
#if IS_ENABLED(CONFIG_ARM64)
/* 6795 */
#define MMDVFS_ENABLE	1
#else
/* 6595 */
#define MMDVFS_ENABLE	0
#endif

#if MMDVFS_ENABLE
#ifndef MMDVFS_STANDALONE
#include <mach/fliper.h>
#endif
#endif

/* WQHD MMDVFS SWITCH */
#define MMDVFS_ENABLE_WQHD	0

#define MMDVFS_GPU_LOADING_NUM	30
#define MMDVFS_GPU_LOADING_START_INDEX	10
#define MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS	100
#define MMDVFS_GPU_LOADING_THRESHOLD	18

/* enable WQHD defalt 1.0v */
/* #define MMDVFS_WQHD_1_0V */

#if (MMDVFS_GPU_LOADING_START_INDEX >= MMDVFS_GPU_LOADING_NUM)
#error "start index too large"
#endif

/* mmdvfs MM sizes */
#define MMDVFS_PIXEL_NUM_720P	(1280 * 720)
#define MMDVFS_PIXEL_NUM_2160P	(3840 * 2160)
#define MMDVFS_PIXEL_NUM_1080P	(2100 * 1300)
#define MMDVFS_PIXEL_NUM_2M		(2100 * 1300)
#define MMDVFS_PIXEL_NUM_13M		(13000000)
#define MMDVFS_PIXEL_NUM_16M		(16000000)

/* lcd size */
typedef enum {
	MMDVFS_LCD_SIZE_HD, MMDVFS_LCD_SIZE_FHD, MMDVFS_LCD_SIZE_WQHD, MMDVFS_LCD_SIZE_END_OF_ENUM
} mmdvfs_lcd_size_enum;

#if defined(MMDVFS_E1) && defined(SMI_J)
/* For UT only */
#define KIR_MM (KIR_MM_16MCAM)
#define mt_dfs_general_pll(a, b) {}
#endif /* defined(MMDVFS_E1) && defined(SMI_J) */

#define MMDVFS_PIXEL_NUM_SENSOR_FULL (MMDVFS_PIXEL_NUM_13M)

/* mmdvfs display sizes */
#define MMDVFS_DISPLAY_SIZE_FHD	(1920 * 1216)
#define MMDVFS_DISPLAY_SIZE_HD	(1280 * 720)

#define MMDVFS_CLK_SWITCH_CB_MAX 16
#define MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX 20


static int notify_cb_func_checked(clk_switch_cb func, int ori_mmsys_clk_mode,
int update_mmsys_clk_mode, char *msg);
static int mmdfvs_adjust_mmsys_clk_by_hopping(int clk_mode);
static int mmdvfs_set_step_with_mmsys_clk(MTK_SMI_BWC_SCEN scenario, mmdvfs_voltage_enum step,
int mmsys_clk_mode);
static void notify_mmsys_clk_change(int ori_mmsys_clk_mode, int update_mmsys_clk_mode);
static int mmsys_clk_change_notify_checked(clk_switch_cb func, int ori_mmsys_clk_mode,
int update_mmsys_clk_mode, char *msg);
static mmdvfs_voltage_enum determine_current_mmsys_clk(void);
static int get_venc_step(int venc_resolution);
static int get_vr_step(int sensor_size, int camera_mode);
static int get_ext_disp_step(mmdvfs_lcd_size_enum disp_resolution);
static int query_vr_step(MTK_MMDVFS_CMD *query_cmd);

#if defined(MMDVFS_E1)
static int get_smvr_step_avc(int resolution, int is_p_mode, int fps);
static int get_smvr_step_hevc(int resolution, int is_p_mode, int fps);
static int get_smvr_step(int is_hevc, int resolution, int is_p_mode, int fps);
#endif /* MMDVFS_E1 */

static int is_cam_monior_work;


static clk_switch_cb quick_mmclk_cbs[MMDVFS_CLK_SWITCH_CB_MAX];
static clk_switch_cb notify_cb_func;
static clk_switch_cb notify_cb_func_nolock;
static int current_mmsys_clk = MMSYS_CLK_MEDIUM;

/* + 1 for MMDVFS_CAM_MON_SCEN */
static mmdvfs_voltage_enum g_mmdvfs_scenario_voltage[MMDVFS_SCEN_COUNT] = {
MMDVFS_VOLTAGE_DEFAULT};
static mmdvfs_voltage_enum g_mmdvfs_current_step;
static unsigned int g_mmdvfs_concurrency;
static MTK_SMI_BWC_MM_INFO *g_mmdvfs_info;
static MTK_MMDVFS_CMD g_mmdvfs_cmd;

/* mmdvfs timer for monitor gpu loading */
typedef struct {
	/* linux timer */
	struct timer_list timer;

	/* work q */
	struct workqueue_struct *work_queue;
	struct work_struct work;

	/* data payload */
	unsigned int gpu_loadings[MMDVFS_GPU_LOADING_NUM];
	int gpu_loading_index;
} mmdvfs_gpu_monitor_struct;

typedef struct {
	spinlock_t scen_lock;
	int is_mhl_enable;
	int is_wfd_enable;
	mmdvfs_gpu_monitor_struct gpu_monitor;

} mmdvfs_context_struct;

/* mmdvfs_query() return value, remember to sync with user space */
typedef enum {
	MMDVFS_STEP_LOW = 0, MMDVFS_STEP_HIGH,

	MMDVFS_STEP_LOW2LOW, /* LOW */
	MMDVFS_STEP_HIGH2LOW, /* LOW */
	MMDVFS_STEP_LOW2HIGH, /* HIGH */
	MMDVFS_STEP_HIGH2HIGH,
/* HIGH */
} mmdvfs_step_enum;



static mmdvfs_context_struct g_mmdvfs_mgr_cntx;
static mmdvfs_context_struct * const g_mmdvfs_mgr = &g_mmdvfs_mgr_cntx;

static mmdvfs_lcd_size_enum mmdvfs_get_lcd_resolution(void)
{
	int lcd_resolution = DISP_GetScreenWidth() * DISP_GetScreenHeight();
	mmdvfs_lcd_size_enum result = MMDVFS_LCD_SIZE_HD;

	if (lcd_resolution <= MMDVFS_DISPLAY_SIZE_HD)
		result = MMDVFS_LCD_SIZE_HD;
	else if (lcd_resolution <= MMDVFS_DISPLAY_SIZE_FHD)
		result = MMDVFS_LCD_SIZE_FHD;
	else
		result = MMDVFS_LCD_SIZE_WQHD;

	return result;
}

static mmdvfs_voltage_enum mmdvfs_get_default_step(void)
{
	return MMDVFS_VOLTAGE_LOW;
}

static mmdvfs_voltage_enum mmdvfs_get_current_step(void)
{
	return g_mmdvfs_current_step;
}

static int mmsys_clk_query(MTK_SMI_BWC_SCEN scenario,
MTK_MMDVFS_CMD *cmd)
{
	int step = MMSYS_CLK_MEDIUM;

	unsigned int venc_size;
	MTK_MMDVFS_CMD cmd_default;

	venc_size = g_mmdvfs_info->video_record_size[0]
	* g_mmdvfs_info->video_record_size[1];

	/* use default info */
	if (cmd == NULL) {
		memset(&cmd_default, 0, sizeof(MTK_MMDVFS_CMD));
		cmd_default.camera_mode = MMDVFS_CAMERA_MODE_FLAG_DEFAULT;
		cmd = &cmd_default;
	}

	/* collect the final information */
	if (cmd->sensor_size == 0)
		cmd->sensor_size = g_mmdvfs_cmd.sensor_size;

	if (cmd->sensor_fps == 0)
		cmd->sensor_fps = g_mmdvfs_cmd.sensor_fps;

	if (cmd->camera_mode == MMDVFS_CAMERA_MODE_FLAG_DEFAULT)
		cmd->camera_mode = g_mmdvfs_cmd.camera_mode;

	/* HIGH level scenarios */
	switch (scenario) {
	case SMI_BWC_SCEN_VR:
		if (is_force_max_mmsys_clk())
			step = MMSYS_CLK_HIGH;

		if (cmd->sensor_size >= MMDVFS_PIXEL_NUM_SENSOR_FULL)
			/* 13M high */
			step = MMSYS_CLK_HIGH;
		else if (cmd->camera_mode & (MMDVFS_CAMERA_MODE_FLAG_PIP |
			MMDVFS_CAMERA_MODE_FLAG_STEREO | MMDVFS_CAMERA_MODE_FLAG_IVHDR))
			/* PIP for ISP clock */
			step = MMSYS_CLK_HIGH;
		break;

	case SMI_BWC_SCEN_VR_SLOW:
	case SMI_BWC_SCEN_ICFP:
		step = MMSYS_CLK_HIGH;
		break;

	default:
		break;
	}

	return step;
}

static mmdvfs_voltage_enum mmdvfs_query(MTK_SMI_BWC_SCEN scenario,
MTK_MMDVFS_CMD *cmd)
{
	mmdvfs_voltage_enum step = mmdvfs_get_default_step();
	unsigned int venc_size;
	MTK_MMDVFS_CMD cmd_default;

	venc_size = g_mmdvfs_info->video_record_size[0]
	* g_mmdvfs_info->video_record_size[1];

	/* use default info */
	if (cmd == NULL) {
		memset(&cmd_default, 0, sizeof(MTK_MMDVFS_CMD));
		cmd_default.camera_mode = MMDVFS_CAMERA_MODE_FLAG_DEFAULT;
		cmd = &cmd_default;
	}

	/* collect the final information */
	if (cmd->sensor_size == 0)
		cmd->sensor_size = g_mmdvfs_cmd.sensor_size;

	if (cmd->sensor_fps == 0)
		cmd->sensor_fps = g_mmdvfs_cmd.sensor_fps;

	if (cmd->camera_mode == MMDVFS_CAMERA_MODE_FLAG_DEFAULT)
		cmd->camera_mode = g_mmdvfs_cmd.camera_mode;

	/* HIGH level scenarios */
	switch (scenario) {

	case SMI_BWC_SCEN_VR:
		if (is_force_camera_hpm())
			step = MMDVFS_VOLTAGE_HIGH;
		else
			step = query_vr_step(cmd);
		break;
	case SMI_BWC_SCEN_VR_SLOW:
		{
#if defined(MMDVFS_E1)
			int resolution = 1920 * 1080;
			int is_p_mode = 0;
			int fps = 120;

			step = get_smvr_step(1, resolution, is_p_mode, fps);
#else
			step = MMDVFS_VOLTAGE_HIGH;
#endif
		}
		break;
	case SMI_BWC_SCEN_ICFP:
		step = MMDVFS_VOLTAGE_HIGH;
		break;

	default:
		break;
	}

	return step;
}

/* Check all scenario in HPM and return the corrosponding mmsys
	clk conciguration setting. This is only need in SMI_J since mmysys
	clk can't be configured independently in SMI_E */
static mmdvfs_voltage_enum determine_current_mmsys_clk(void)
{
	int i = 0;
	int final_clk = MMSYS_CLK_MEDIUM;

	for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
		if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH) {
			/* Check the mmsys clk */
			switch (i) {
			case SMI_BWC_SCEN_VR:
			case MMDVFS_CAM_MON_SCEN:
				if (is_force_max_mmsys_clk())
					final_clk = MMSYS_CLK_HIGH;
				else if (g_mmdvfs_cmd.sensor_size >= MMDVFS_PIXEL_NUM_SENSOR_FULL)
					/* 13M high */
					final_clk = MMSYS_CLK_HIGH;
				else if (g_mmdvfs_cmd.camera_mode & (MMDVFS_CAMERA_MODE_FLAG_PIP |
				MMDVFS_CAMERA_MODE_FLAG_STEREO | MMDVFS_CAMERA_MODE_FLAG_IVHDR))
					/* PIP for ISP clock */
					final_clk = MMSYS_CLK_HIGH;
				break;
			case SMI_BWC_SCEN_VR_SLOW:
			case SMI_BWC_SCEN_ICFP:
				final_clk = MMSYS_CLK_HIGH;
				break;
			default:
				break;
			}
		}
	}

	return final_clk;
}


static void mmdvfs_update_cmd(MTK_MMDVFS_CMD *cmd)
{
	if (cmd == NULL)
		return;

	if (cmd->sensor_size)
		g_mmdvfs_cmd.sensor_size = cmd->sensor_size;

	if (cmd->sensor_fps)
		g_mmdvfs_cmd.sensor_fps = cmd->sensor_fps;

	/* MMDVFSMSG("update cm %d\n", cmd->camera_mode); */

	/* if (cmd->camera_mode != MMDVFS_CAMERA_MODE_FLAG_DEFAULT) { */
	g_mmdvfs_cmd.camera_mode = cmd->camera_mode;
	/* } */
}


#ifdef MMDVFS_GPU_MONITOR_ENABLE
static void mmdvfs_timer_callback(unsigned long data)
{
	mmdvfs_gpu_monitor_struct *gpu_monitor =
	(mmdvfs_gpu_monitor_struct *)data;

	unsigned int gpu_loading = 0;

	/* if (mtk_get_gpu_loading(&gpu_loading)) {
		MMDVFSMSG("gpuload %d %ld\n", gpu_loading, jiffies_to_msecs(jiffies));
	*/

	/* store gpu loading into the array */
	gpu_monitor->gpu_loadings[gpu_monitor->gpu_loading_index++]
	= gpu_loading;

	/* fire another timer until the end */
	if (gpu_monitor->gpu_loading_index < MMDVFS_GPU_LOADING_NUM - 1) {
		mod_timer(
		&gpu_monitor->timer,
		jiffies + msecs_to_jiffies(
		MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS));
	} else {
		/* the final timer */
		int i;
		int avg_loading;
		unsigned int sum = 0;

		for (i = MMDVFS_GPU_LOADING_START_INDEX; i
		< MMDVFS_GPU_LOADING_NUM; i++) {
			sum += gpu_monitor->gpu_loadings[i];
		}

		avg_loading = sum / MMDVFS_GPU_LOADING_NUM;

		MMDVFSMSG("gpuload %d AVG %d\n", jiffies_to_msecs(jiffies),
		avg_loading);

		/* drops to low step if the gpu loading is low */
		if (avg_loading <= MMDVFS_GPU_LOADING_THRESHOLD)
			queue_work(gpu_monitor->work_queue, &gpu_monitor->work);
	}

}

static void mmdvfs_gpu_monitor_work(struct work_struct *work)
{
	MMDVFSMSG("WQ %d\n", jiffies_to_msecs(jiffies));
}

static void mmdvfs_init_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;

	/* setup gpu monitor timer */
	setup_timer(gpu_timer, mmdvfs_timer_callback, (unsigned long)gm);

	gm->work_queue = create_singlethread_workqueue("mmdvfs_gpumon");
	INIT_WORK(&gm->work, mmdvfs_gpu_monitor_work);
}
#endif /* MMDVFS_GPU_MONITOR_ENABLE */

/* delay 4 seconds to go LPM to workaround camera ZSD + PIP issue */
static void mmdvfs_cam_work_handler(struct work_struct *work)
{
	/* MMDVFSMSG("CAM handler %d\n", jiffies_to_msecs(jiffies)); */
	mmdvfs_set_step(MMDVFS_CAM_MON_SCEN, mmdvfs_get_default_step());

	spin_lock(&g_mmdvfs_mgr->scen_lock);
	is_cam_monior_work = 0;
	spin_unlock(&g_mmdvfs_mgr->scen_lock);

}

static DECLARE_DELAYED_WORK(g_mmdvfs_cam_work, mmdvfs_cam_work_handler);

static void mmdvfs_stop_cam_monitor(void)
{
	cancel_delayed_work_sync(&g_mmdvfs_cam_work);
}

static void mmdvfs_start_cam_monitor(int scen, int delay_hz)
{
	int delayed_mmsys_state = MMSYS_CLK_MEDIUM;

	mmdvfs_stop_cam_monitor();

	spin_lock(&g_mmdvfs_mgr->scen_lock);
	is_cam_monior_work = 1;
	spin_unlock(&g_mmdvfs_mgr->scen_lock);

	if (current_mmsys_clk == MMSYS_CLK_LOW)
		MMDVFSMSG("Error!Can't switch clk by hopping when CLK is low\n");

	/* Make sure advance feature is in high frequency mode for J1 pr0file */
	if (is_force_max_mmsys_clk())
		delayed_mmsys_state = MMSYS_CLK_HIGH;
	if ((scen == SMI_BWC_SCEN_ICFP || scen == SMI_BWC_SCEN_VR_SLOW || scen == SMI_BWC_SCEN_VR) &&
			(g_mmdvfs_cmd.camera_mode & (MMDVFS_CAMERA_MODE_FLAG_PIP | MMDVFS_CAMERA_MODE_FLAG_STEREO
			| MMDVFS_CAMERA_MODE_FLAG_IVHDR )))
		delayed_mmsys_state = MMSYS_CLK_HIGH;
	else if (current_mmsys_clk == MMSYS_CLK_LOW)
			delayed_mmsys_state = MMSYS_CLK_MEDIUM;
	else
			delayed_mmsys_state = current_mmsys_clk;

	/* MMDVFSMSG("MMDVFS boost:%d\n", delay_hz); */
	mmdvfs_set_step_with_mmsys_clk(MMDVFS_CAM_MON_SCEN, MMDVFS_VOLTAGE_HIGH, delayed_mmsys_state);

	schedule_delayed_work(&g_mmdvfs_cam_work, delay_hz * HZ);
}

#if MMDVFS_ENABLE_WQHD

static void mmdvfs_start_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;

	gm->gpu_loading_index = 0;
	memset(gm->gpu_loadings, 0, sizeof(unsigned int) * MMDVFS_GPU_LOADING_NUM);

	mod_timer(gpu_timer, jiffies + msecs_to_jiffies(MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS));
}

static void mmdvfs_stop_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;

	/* flush workqueue */
	flush_workqueue(gm->work_queue);
	/* delete timer */
	del_timer(gpu_timer);
}

#endif /* MMDVFS_ENABLE_WQHD */

static void mmdvfs_vcorefs_request_dvfs_opp(int mm_kicker, int mm_dvfs_opp)
{
	int vcore_enable = 0;
#ifdef MMDVFS_E1
	vcore_enable = is_vcorefs_feature_enable();
#else
	vcore_enable = is_vcorefs_can_work();
#endif /* MMDVFS_E1 */

	if (vcore_enable != 1) {
		MMDVFSMSG("Vcore disable: is_vcorefs_can_work = %d, (%d, %d)\n", vcore_enable, mm_kicker, mm_dvfs_opp);
	} else {
		/* MMDVFSMSG("Vcore trigger: is_vcorefs_can_work = %d, (%d, %d)\n", vcore_enable,
		mm_kicker, mm_dvfs_opp); */
		vcorefs_request_dvfs_opp(mm_kicker, mm_dvfs_opp);
	}
}

int mmdvfs_set_step(MTK_SMI_BWC_SCEN scenario, mmdvfs_voltage_enum step)
{
	/* Default mmsys clk step MMSYS_CLK_MEDIUM only apply to SMI_J,
		In SMI_E1 project, the mmsys clk is determined by voltage directly
		and can't be configure indepdently.*/
	return mmdvfs_set_step_with_mmsys_clk(scenario, step, MMSYS_CLK_MEDIUM);
}

static int get_venc_step(int venc_resolution)
{
	int lpm_size_limit = 0;
	int venc_step = MMDVFS_VOLTAGE_LOW;

	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD)
		/* initialize the venc_size_limit */
		lpm_size_limit = 1920 * 1080;
	else
		/* initialize the venc_size_limit */
		lpm_size_limit = 4096 * 1716;

	/* Check recording video resoltuion */
	if (venc_resolution >= lpm_size_limit)
		venc_step = MMDVFS_VOLTAGE_HIGH;

	return venc_step;
}

static int get_ext_disp_step(mmdvfs_lcd_size_enum lcd_size)
{
	int result = MMDVFS_VOLTAGE_HIGH;
#ifdef MMDVFS_E1
	if (lcd_size != MMDVFS_LCD_SIZE_WQHD)
		result = MMDVFS_VOLTAGE_LOW;
#else
	if (lcd_size == MMDVFS_LCD_SIZE_HD)
		result = MMDVFS_VOLTAGE_LOW;
#endif /* MMDVFS_E1 */

	return result;
}

/* If the query_cmd is null, we will not check the information of sensor size
and fps since the information is set after profile change */
static int get_vr_step(int sensor_size, int camera_mode)
{
	unsigned int hpm_cam_mode = 0;

	int	vr_step = MMDVFS_VOLTAGE_LOW;

	/* initialize the venc_size_limit */
	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {
		/* All camera feature triggers HPM mode */
		hpm_cam_mode = (MMDVFS_CAMERA_MODE_FLAG_PIP | MMDVFS_CAMERA_MODE_FLAG_VFB
		| MMDVFS_CAMERA_MODE_FLAG_EIS_2_0 | MMDVFS_CAMERA_MODE_FLAG_IVHDR |
		MMDVFS_CAMERA_MODE_FLAG_STEREO);

	} else {
		hpm_cam_mode = (MMDVFS_CAMERA_MODE_FLAG_PIP | MMDVFS_CAMERA_MODE_FLAG_VFB
		| MMDVFS_CAMERA_MODE_FLAG_EIS_2_0 | MMDVFS_CAMERA_MODE_FLAG_STEREO
		| MMDVFS_CAMERA_MODE_FLAG_IVHDR);
	}

	/* Check sensor size */
	if (sensor_size >= MMDVFS_PIXEL_NUM_SENSOR_FULL)
		vr_step = MMDVFS_VOLTAGE_HIGH;

	/* Check camera mode flag */
	if (camera_mode & hpm_cam_mode)
		vr_step = MMDVFS_VOLTAGE_HIGH;

	return vr_step;
}


static int query_vr_step(MTK_MMDVFS_CMD *query_cmd)
{
	if (query_cmd == NULL)
		return MMDVFS_VOLTAGE_LOW;
	else
		return get_vr_step(query_cmd->sensor_size, query_cmd->camera_mode);

}

#if defined(MMDVFS_E1)
static int get_smvr_step_avc(int resolution, int is_p_mode, int fps)
{
	/* Spec
	HPM:
		AVC 1080P, fps >= 60
	LPM:
		AVC 720p60
	*/
	if (resolution >= 1920 * 1080 && fps >= 60)
		return MMDVFS_VOLTAGE_HIGH;
	else
		return MMDVFS_VOLTAGE_LOW;
}

static int get_smvr_step_hevc(int resolution, int is_p_mode, int fps)
{
	/* Spec
	HPM:
		HEVC 1080p (Q), fps >= 60
		HEVC 1080p (P), fps >= 120
		HEVC 720p(Q), fps >= 120
	LPM:
		HEVC 1080p60 (P) ?
		HEVC 720p120 (P)
		HEVC 720p60 (Q)
	*/

	if (is_p_mode)
		if (resolution >= 1920 * 1080 && fps >= 120)
			return MMDVFS_VOLTAGE_HIGH;
		else
			return MMDVFS_VOLTAGE_LOW;
	else
		/* Quality mode*/
		if (resolution >= 1920 * 1080 && fps >= 60)
				return MMDVFS_VOLTAGE_HIGH;
		else if (resolution >= 1080 * 720 && fps >= 120)
				return MMDVFS_VOLTAGE_HIGH;
		else
				return MMDVFS_VOLTAGE_LOW;
}

static int get_smvr_step(int is_hevc, int resolution, int is_p_mode, int fps)
{
	if (is_hevc)
		return get_smvr_step_hevc(resolution, is_p_mode, fps);
	else
		return get_smvr_step_avc(resolution, is_p_mode, fps);
}
#endif /* MMDVFS_E1 */

/* mmsys_clk_mode_request only supported in SMI_J,
In SMI_E1 project, the mmsys clk is determined by voltage directly
and can't be configure indepdently. */
int mmdvfs_set_step_with_mmsys_clk(MTK_SMI_BWC_SCEN smi_scenario, mmdvfs_voltage_enum step, int mmsys_clk_mode_request)
{
	int i, scen_index;
	unsigned int concurrency;
	unsigned int scenario = smi_scenario;
	mmdvfs_voltage_enum final_step = mmdvfs_get_default_step();
	int mmsys_clk_step = MMSYS_CLK_MEDIUM;
	int mmsys_clk_mode = mmsys_clk_mode_request;

	if (is_mmdvfs_disabled()) {
		MMDVFSMSG("MMDVFS is disable, request denalied; scen:%d, vol:%d, clk:%d\n",
		scenario, step, mmsys_clk_mode_request);
		return 0;
	}

#if !defined(MMDVFS_E1)
	/* Only everest allows VP as MMDVFS client */
	if (scenario == SMI_BWC_SCEN_VENC || scenario == SMI_BWC_SCEN_VP
		|| scenario == MMDVFS_SCEN_VP_HIGH_RESOLUTION)
		return 0;
#else
	if (scenario == MMDVFS_SCEN_VP_HIGH_RESOLUTION) {
		if ((!g_mmdvfs_mgr->is_wfd_enable) && (!g_mmdvfs_mgr->is_mhl_enable)) {
			MMDVFSMSG("Reject HPM request: WFD is off\n");
			MMDVFSMSG("VP 4K can only trigger HPM with WFD(%d) or MHL(%d)\n",
			g_mmdvfs_mgr->is_wfd_enable, g_mmdvfs_mgr->is_mhl_enable);
			return 0;
		}
	}
#endif

	if (step == MMDVFS_VOLTAGE_DEFAULT_STEP)
		step = final_step;

#if !MMDVFS_ENABLE
	return 0;
#endif

	/* MMDVFSMSG("MMDVFS set voltage scen %d step %d\n", scenario, step); */

	if ((scenario >= (MTK_SMI_BWC_SCEN)MMDVFS_SCEN_COUNT) || (scenario
	< SMI_BWC_SCEN_NORMAL)) {
		MMDVFSERR("invalid scenario\n");
		return -1;
	}

	/* dump information */
	/* mmdvfs_dump_info(); */

	/* go through all scenarios to decide the final step */
	scen_index = (int)scenario;

	spin_lock(&g_mmdvfs_mgr->scen_lock);

	g_mmdvfs_scenario_voltage[scen_index] = step;

	concurrency = 0;
	for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
		if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH)
			concurrency |= 1 << i;
	}

	/* one high = final high */
	for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
		if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH) {
			final_step = MMDVFS_VOLTAGE_HIGH;
			break;
		}
	}

	mmsys_clk_step = determine_current_mmsys_clk();
	if (mmsys_clk_mode_request == MMSYS_CLK_MEDIUM && mmsys_clk_step == MMSYS_CLK_HIGH)
		mmsys_clk_mode = MMSYS_CLK_HIGH;
	else
		mmsys_clk_mode = mmsys_clk_mode_request;

	g_mmdvfs_current_step = final_step;

	spin_unlock(&g_mmdvfs_mgr->scen_lock);

#if	MMDVFS_ENABLE

	/* call vcore dvfs API */
	/* MMDVFSMSG("FHD %d\n", final_step); */

	if (final_step == MMDVFS_VOLTAGE_HIGH) {
		#ifdef MMDVFS_E1
			mmdvfs_vcorefs_request_dvfs_opp(KIR_MM, OPPI_PERF);
			mmdfvs_adjust_mmsys_clk_by_hopping(MMSYS_CLK_HIGH);
		#else
			if (scenario == MMDVFS_SCEN_MHL)
				mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_MHL, OPPI_PERF);
			else if (scenario == SMI_BWC_SCEN_WFD)
				mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_WFD, OPPI_PERF);
			else {
				mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_16MCAM, OPPI_PERF);
				mmdfvs_adjust_mmsys_clk_by_hopping(mmsys_clk_mode);
			}
		#endif /* MMDVFS_E1 */
	}	else{
		#ifdef MMDVFS_E1
			mmdfvs_adjust_mmsys_clk_by_hopping(MMSYS_CLK_MEDIUM);
			mmdvfs_vcorefs_request_dvfs_opp(KIR_MM, OPPI_UNREQ);
		#else
		if (scenario == MMDVFS_SCEN_MHL)
			mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_MHL, OPPI_UNREQ);
		else if (scenario == SMI_BWC_SCEN_WFD)
			mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_WFD, OPPI_UNREQ);
		else {
		  /* must lower the mmsys clk before enter LPM mode */
			mmdfvs_adjust_mmsys_clk_by_hopping(MMSYS_CLK_MEDIUM);
			mmdvfs_vcorefs_request_dvfs_opp(KIR_MM_16MCAM, OPPI_UNREQ);
		}
		#endif /* MMDVFS_E1 */
	}
#endif /* MMDVFS_ENABLE */

	MMDVFSMSG("Set vol scen:%d,step:%d,final:%d(0x%x),CMD(%d,%d,0x%x),INFO(%d,%d),CLK:%d\n",
	scenario, step, final_step, concurrency,
	g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps, g_mmdvfs_cmd.camera_mode,
	g_mmdvfs_info->video_record_size[0], g_mmdvfs_info->video_record_size[1],
	current_mmsys_clk);


	return 0;
}

void mmdvfs_handle_cmd(MTK_MMDVFS_CMD *cmd)
{
#if !MMDVFS_ENABLE
	return;
#endif

	/* MMDVFSMSG("MMDVFS handle cmd %u s %d\n", cmd->type, cmd->scen); */

	switch (cmd->type) {
	case MTK_MMDVFS_CMD_TYPE_SET:
		/* save cmd */
		mmdvfs_update_cmd(cmd);

		if (!(g_mmdvfs_concurrency & (1 << cmd->scen))) {
			MMDVFSMSG("invalid set scen %d\n", cmd->scen);
			cmd->ret = -1;
		} else {
			cmd->ret = mmdvfs_set_step_with_mmsys_clk(cmd->scen,
			mmdvfs_query(cmd->scen, cmd), mmsys_clk_query(cmd->scen, cmd));
		}
		break;

	case MTK_MMDVFS_CMD_TYPE_QUERY: { /* query with some parameters */
		{
			mmdvfs_voltage_enum query_voltage = mmdvfs_query(cmd->scen, cmd);

			mmdvfs_voltage_enum current_voltage =	mmdvfs_get_current_step();

			if (current_voltage < query_voltage) {
				cmd->ret = (unsigned int)MMDVFS_STEP_LOW2HIGH;
			} else if (current_voltage > query_voltage) {
				cmd->ret = (unsigned int)MMDVFS_STEP_HIGH2LOW;
			} else {
				cmd->ret
				= (unsigned int)(query_voltage
				== MMDVFS_VOLTAGE_HIGH
							 ? MMDVFS_STEP_HIGH2HIGH
							 : MMDVFS_STEP_LOW2LOW);
			}
		}

		/* MMDVFSMSG("query %d\n", cmd->ret); */
		/* cmd->ret = (unsigned int)query_voltage; */
		break;
	}

	default:
		MMDVFSMSG("invalid mmdvfs cmd\n");
		BUG();
		break;
	}
}

void mmdvfs_notify_scenario_exit(MTK_SMI_BWC_SCEN scen)
{
#if !MMDVFS_ENABLE
	return;
#endif

	/* MMDVFSMSG("leave %d\n", scen); */
	if (scen == SMI_BWC_SCEN_WFD)
		g_mmdvfs_mgr->is_wfd_enable = 0;

	if ((scen == SMI_BWC_SCEN_VR) || (scen == SMI_BWC_SCEN_VR_SLOW) || (scen == SMI_BWC_SCEN_ICFP))
		mmdvfs_start_cam_monitor(scen, 8);

	/* reset scenario voltage to default when it exits */
	mmdvfs_set_step(scen, mmdvfs_get_default_step());
}

void mmdvfs_notify_scenario_enter(MTK_SMI_BWC_SCEN scen)
{
	mmdvfs_lcd_size_enum lcd_size_detected = MMDVFS_LCD_SIZE_WQHD;

	lcd_size_detected = mmdvfs_get_lcd_resolution();
#if !MMDVFS_ENABLE
	return;
#endif

	if (scen == SMI_BWC_SCEN_VR && is_force_camera_hpm()) {
		/* currently we set mmsys clk medium in default
		when force_camera_hpm is enabled */
		int mmsys_clk_request = MMSYS_CLK_MEDIUM;

		if (is_force_max_mmsys_clk())
			mmsys_clk_request = MMSYS_CLK_HIGH;

		mmdvfs_set_step_with_mmsys_clk(scen, MMDVFS_VOLTAGE_HIGH, mmsys_clk_request);
		return;
	}

	/* Leave display idle mode before set scenario */
	if (current_mmsys_clk == MMSYS_CLK_LOW && scen != SMI_BWC_SCEN_NORMAL)
		mmdvfs_raise_mmsys_by_mux();

	/* Boost for ISP related scenario */
	if ((scen == SMI_BWC_SCEN_VR) || (scen == SMI_BWC_SCEN_VR_SLOW) || (scen == SMI_BWC_SCEN_ICFP))
		mmdvfs_start_cam_monitor(scen, 8);

	switch (scen) {
	case SMI_BWC_SCEN_VENC:
		if (g_mmdvfs_concurrency & (1 << SMI_BWC_SCEN_VR))
			mmdvfs_set_step(scen, get_venc_step(g_mmdvfs_info->video_record_size[0] *
			g_mmdvfs_info->video_record_size[1]));
		break;
	case SMI_BWC_SCEN_WFD:
		g_mmdvfs_mgr->is_wfd_enable = 1;
		mmdvfs_set_step(scen, get_ext_disp_step(lcd_size_detected));
		break;
	case SMI_BWC_SCEN_VR:
		{
			mmdvfs_voltage_enum vr_step = MMDVFS_VOLTAGE_LOW;
			mmdvfs_voltage_enum venc_step = MMDVFS_VOLTAGE_LOW;

			vr_step = get_vr_step(g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.camera_mode);

			if (g_mmdvfs_concurrency & (1 << SMI_BWC_SCEN_VENC))
				venc_step = get_venc_step(g_mmdvfs_info->video_record_size[0] *
				g_mmdvfs_info->video_record_size[1]);

			if (vr_step == MMDVFS_VOLTAGE_HIGH || venc_step == MMDVFS_VOLTAGE_HIGH) {
				if (g_mmdvfs_cmd.camera_mode & (MMDVFS_CAMERA_MODE_FLAG_VFB |
					MMDVFS_CAMERA_MODE_FLAG_EIS_2_0))
					/* Only set HPM mode for EMI BW requirement, don't care the mm clock */
					mmdvfs_set_step_with_mmsys_clk(scen, MMDVFS_VOLTAGE_HIGH,
						MMSYS_CLK_MEDIUM);
				else
					mmdvfs_set_step_with_mmsys_clk(scen, MMDVFS_VOLTAGE_HIGH,
						MMSYS_CLK_HIGH);
				}
			break;
		}
	case SMI_BWC_SCEN_VR_SLOW:
	case SMI_BWC_SCEN_ICFP:
		mmdvfs_set_step_with_mmsys_clk(scen, MMDVFS_VOLTAGE_HIGH, MMSYS_CLK_HIGH);
		break;

	default:
		break;
	}
}

void mmdvfs_init(MTK_SMI_BWC_MM_INFO *info)
{
#if !MMDVFS_ENABLE
	return;
#endif

	spin_lock_init(&g_mmdvfs_mgr->scen_lock);
	/* set current step as the default step */
	g_mmdvfs_current_step = mmdvfs_get_default_step();

	g_mmdvfs_info = info;

#ifdef MMDVFS_GPU_MONITOR_ENABLE
	mmdvfs_init_gpu_monitor(&g_mmdvfs_mgr->gpu_monitor);
#endif /* MMDVFS_GPU_MONITOR_ENABLE */
}

void mmdvfs_mhl_enable(int enable)
{
	g_mmdvfs_mgr->is_mhl_enable = enable;

	if (enable)
		mmdvfs_set_step(MMDVFS_SCEN_MHL, get_ext_disp_step(mmdvfs_get_lcd_resolution()));
	else
		mmdvfs_set_step(MMDVFS_SCEN_MHL, MMDVFS_VOLTAGE_DEFAULT_STEP);
}

void mmdvfs_notify_scenario_concurrency(unsigned int u4Concurrency)
{
	/*
	 * DO NOT CALL VCORE DVFS API HERE. THIS FUNCTION IS IN SMI SPIN LOCK.
	 */

	/* raise EMI monitor BW threshold in VP, VR, VR SLOW motion cases to
	make sure vcore stay MMDVFS level as long as possible */
	if (u4Concurrency & ((1 << SMI_BWC_SCEN_VP) | (1 << SMI_BWC_SCEN_VR)
	| (1 << SMI_BWC_SCEN_VR_SLOW))) {
#if MMDVFS_ENABLE
		/* MMDVFSMSG("fliper high\n"); */
		/* fliper_set_bw(BW_THRESHOLD_HIGH); */
#endif
	} else {
#if MMDVFS_ENABLE
		/* MMDVFSMSG("fliper normal\n"); */
		/* fliper_restore_bw(); */
#endif
	}

	g_mmdvfs_concurrency = u4Concurrency;
}

int mmdvfs_is_default_step_need_perf(void)
{
	if (mmdvfs_get_default_step() == MMDVFS_VOLTAGE_LOW)
		return 0;
	else
		return 1;
}

/* switch MM CLK callback from VCORE DVFS driver */
void mmdvfs_mm_clock_switch_notify(int is_before, int is_to_high)
{
	/* for WQHD 1.0v, we have to dynamically switch DL/DC */
#ifdef MMDVFS_WQHD_1_0V
	int session_id;

	if (mmdvfs_get_lcd_resolution() != MMDVFS_LCD_SIZE_WQHD)
		return;

	session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);

	if (!is_before && is_to_high) {
		MMDVFSMSG("DL\n");
		/* nonblocking switch to direct link after HPM */
		primary_display_switch_mode_for_mmdvfs(DISP_SESSION_DIRECT_LINK_MODE, session_id, 0);
	} else if (is_before && !is_to_high) {
		/* BLOCKING switch to decouple before switching to LPM */
		MMDVFSMSG("DC\n");
		primary_display_switch_mode_for_mmdvfs(DISP_SESSION_DECOUPLE_MODE, session_id, 1);
	}
#endif /* MMDVFS_WQHD_1_0V */
}

static int mmdfvs_adjust_mmsys_clk_by_hopping(int clk_mode)
{
	int freq_hopping_disable = is_mmdvfs_freq_hopping_disabled();

	int result = 0;

	if (clk_mode == MMSYS_CLK_HIGH) {
		if (current_mmsys_clk == MMSYS_CLK_LOW) {
			MMDVFSMSG("Doesn't allow mmsys clk adjust from low to high!\n");
	  } else if (!freq_hopping_disable && current_mmsys_clk != MMSYS_CLK_HIGH) {
			#ifdef MMDVFS_E1
				/* MMDVFSMSG("IMGPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL6, 0x114EC5); */
				mt_dfs_general_pll(FH_PLL6, 0x114EC5); /* IMGPLL */
				/* MMDVFSMSG("CODECPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL8, 0x130000); */
				mt_dfs_general_pll(FH_PLL8, 0x130000); /* CODECPLL */
				/* MMDVFSMSG("VDECPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL0, 0x133B33); */
				mt_dfs_general_pll(FH_PLL0, 0x133B33); /* VDECPLL */
			#else
				/* MMDVFSMSG("Freq hopping: DSS: %d\n", 0xE0000);*/
				mt_dfs_vencpll(0xE0000);
				/* For 182Mhz DISP idle mode */
				notify_cb_func_checked(notify_cb_func, current_mmsys_clk, MMSYS_CLK_HIGH,
				"notify_cb_func");
			#endif
			/* For common clients */
			notify_mmsys_clk_change(current_mmsys_clk, MMSYS_CLK_HIGH);
			current_mmsys_clk = MMSYS_CLK_HIGH;
		} else {
			if (freq_hopping_disable)
				MMDVFSMSG("Freq hopping disable, not trigger: DSS: %d\n", 0xE0000);
		}
		result = 1;
	} else if (clk_mode == MMSYS_CLK_MEDIUM) {
		if (!freq_hopping_disable && current_mmsys_clk != MMSYS_CLK_MEDIUM) {
			#ifdef MMDVFS_E1
				/* MMDVFSMSG("VDECPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL0, 0xD0000); */
				mt_dfs_general_pll(FH_PLL0, 0xD0000); /* VDECPLL */
				/* MMDVFSMSG("CODECPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL8, 0xC8000); */
				mt_dfs_general_pll(FH_PLL8, 0xC8000); /* CODECPLL */
				/* MMDVFSMSG("IMGPLL Freq hopping: FH_PLL: %d, DSS: %d\n", FH_PLL6, 0xC8000); */
				mt_dfs_general_pll(FH_PLL6, 0xC8000); /* IMGPLL */

			#else
				/* MMDVFSMSG("Freq hopping: DSS: %d\n", 0xB0000); */
				mt_dfs_vencpll(0xB0000);
				notify_cb_func_checked(notify_cb_func, current_mmsys_clk, MMSYS_CLK_MEDIUM,
				"notify_cb_func");
			#endif
			/* For common clients */
			notify_mmsys_clk_change(current_mmsys_clk, MMSYS_CLK_MEDIUM);
			current_mmsys_clk = MMSYS_CLK_MEDIUM;
		} else {
			if (freq_hopping_disable)
				MMDVFSMSG("Freq hopping disable, not trigger: DSS: %d\n", 0xB0000);
		}
		result = 1;
	} else if (clk_mode == MMSYS_CLK_LOW) {
		MMDVFSMSG("Doesn't support MMSYS_CLK_LOW with hopping in this platform\n");
		result = 1;
	} else {
		MMDVFSMSG("Don't change CLK: mode=%d\n", clk_mode);
		result = 0;
	}

	return result;
}

int mmdvfs_raise_mmsys_by_mux(void)
{
#ifdef MMDVFS_E1
	return 0;
#endif

	if (is_mmdvfs_freq_mux_disabled())
		return 0;

	notify_cb_func_checked(notify_cb_func, current_mmsys_clk, MMSYS_CLK_MEDIUM,
	"notify_cb_func");
	current_mmsys_clk = MMSYS_CLK_MEDIUM;
	return 1;

}

int mmdvfs_lower_mmsys_by_mux(void)
{
#ifdef MMDVFS_E1
	return 0;
#endif
	if (is_mmdvfs_freq_mux_disabled())
		return 0;

	if (notify_cb_func != NULL && current_mmsys_clk != MMSYS_CLK_HIGH) {
		notify_cb_func(current_mmsys_clk, MMSYS_CLK_LOW);
		current_mmsys_clk = MMSYS_CLK_LOW;
	}	else{
		MMDVFSMSG("lower_cb_func has not been registered");
		return 0;
	}
	return 1;

}

int register_mmclk_switch_cb(clk_switch_cb notify_cb,
clk_switch_cb notify_cb_nolock)
{
	notify_cb_func = notify_cb;
	notify_cb_func_nolock = notify_cb_nolock;

	return 1;
}

static int notify_cb_func_checked(clk_switch_cb func, int ori_mmsys_clk_mode, int update_mmsys_clk_mode, char *msg)
{

	if (is_mmdvfs_freq_mux_disabled()) {
		MMDVFSMSG("notify_cb_func is disabled, not invoked: %s, (%d,%d)\n", msg, ori_mmsys_clk_mode,
			update_mmsys_clk_mode);
		return 0;
	}
	if (func == NULL) {
		MMDVFSMSG("notify_cb_func is NULL, not invoked: %s, (%d,%d)\n", msg, ori_mmsys_clk_mode,
		update_mmsys_clk_mode);
	} else {
		if (ori_mmsys_clk_mode != update_mmsys_clk_mode)
			MMDVFSMSG("notify_cb_func: %s, (%d,%d)\n", msg, ori_mmsys_clk_mode, update_mmsys_clk_mode);

		func(ori_mmsys_clk_mode, update_mmsys_clk_mode);
		return 1;
	}
	return 0;
}

/* This desing is only for CLK Mux switch relate flows */
int mmdvfs_notify_mmclk_switch_request(int event)
{
	int i = 0;
	MTK_SMI_BWC_SCEN current_smi_scenario = smi_get_current_profile();

#ifdef MMDVFS_E1
	return 0;
#endif

	/* Don't get the lock since there is no need to synchronize the is_cam_monior_work here*/
	if (is_cam_monior_work != 0) {
		/* MMDVFSMSG("Doesn't handle disp request when cam monitor is active\n"); */
		return 0;
	}
	/* MMDVFSMSG("mmclk_switch_request: event=%d, current=%d", event, current_smi_scenario); */

	/* Only in UI idle modw or VP 1 layer scenario */
	/* we can lower the mmsys clock */
	if (event == MMDVFS_EVENT_UI_IDLE_ENTER && current_smi_scenario == SMI_BWC_SCEN_NORMAL) {
		for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
			if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH) {
				MMDVFSMSG("Doesn't switch to low mmsys clk; vore is still in HPM mode");
				return 0;
			}
		}

		/* call back from DISP so we don't need use DISP lock here */
		if (current_mmsys_clk != MMSYS_CLK_HIGH) {
			/* Only disable VENC pll when clock is in 286MHz */
			notify_cb_func_checked(notify_cb_func_nolock, current_mmsys_clk, MMSYS_CLK_LOW,
			"notify_cb_func_nolock");
			current_mmsys_clk = MMSYS_CLK_LOW;
			return 1;
		}
	} else if (event == MMDVFS_EVENT_OVL_SINGLE_LAYER_EXIT || event == MMDVFS_EVENT_UI_IDLE_EXIT) {
		if (current_mmsys_clk != MMSYS_CLK_HIGH) {
			/* call back from DISP so we don't need use DISP lock here */
			notify_cb_func_checked(notify_cb_func_nolock, current_mmsys_clk, MMSYS_CLK_MEDIUM,
			"notify_cb_func_nolock");
			current_mmsys_clk = MMSYS_CLK_MEDIUM;
			return 1;
		}
	} else if (event == MMDVFS_EVENT_OVL_SINGLE_LAYER_ENTER && SMI_BWC_SCEN_VP) {
		/* call back from DISP so we don't need use DISP lock here */
		if (current_mmsys_clk != MMSYS_CLK_HIGH) {
			notify_cb_func_checked(notify_cb_func_nolock, current_mmsys_clk, MMSYS_CLK_LOW,
			"notify_cb_func_nolock");
			current_mmsys_clk = MMSYS_CLK_LOW;
			return 1;
		}
	}
	return 0;
}


int mmdvfs_register_mmclk_switch_cb(clk_switch_cb notify_cb, int mmdvfs_client_id)
{
	if (mmdvfs_client_id >= 0 && mmdvfs_client_id < MMDVFS_CLK_SWITCH_CB_MAX) {
		quick_mmclk_cbs[mmdvfs_client_id] = notify_cb;
	} else{
		MMDVFSMSG("clk_switch_cb register failed: id=%d\n", mmdvfs_client_id);
		return 1;
	}
	return 0;
}

static int mmsys_clk_change_notify_checked(clk_switch_cb func, int ori_mmsys_clk_mode,
int update_mmsys_clk_mode, char *msg)
{
	if (func == NULL) {
		MMDVFSMSG("notify_cb_func is NULL, not invoked: %s, (%d,%d)\n", msg, ori_mmsys_clk_mode,
		update_mmsys_clk_mode);
	} else {
		MMDVFSMSG("notify_cb_func: %s, (%d,%d)\n", msg, ori_mmsys_clk_mode, update_mmsys_clk_mode);
		func(ori_mmsys_clk_mode, update_mmsys_clk_mode);
		return 1;
	}
	return 0;
}

static void notify_mmsys_clk_change(int ori_mmsys_clk_mode, int update_mmsys_clk_mode)
{
	int i = 0;

	char msg[MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX] = "";

	for (i = 0; i < MMDVFS_CLK_SWITCH_CB_MAX; i++) {
		snprintf(msg, MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX, "id=%d", i);
		if (quick_mmclk_cbs[i] != NULL)
			mmsys_clk_change_notify_checked(quick_mmclk_cbs[i], ori_mmsys_clk_mode,
			update_mmsys_clk_mode, msg);
	}
}


void dump_mmdvfs_info(void)
{
	int i = 0;

	MMDVFSMSG("MMDVFS dump: CMD(%d,%d,0x%x),INFO VR(%d,%d),CLK: %d\n",
	g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps, g_mmdvfs_cmd.camera_mode,
	g_mmdvfs_info->video_record_size[0], g_mmdvfs_info->video_record_size[1],
	current_mmsys_clk);

	for (i = 0; i < MMDVFS_SCEN_COUNT; i++)
		MMDVFSMSG("Secn:%d,vol-step:%d\n", i, g_mmdvfs_scenario_voltage[i]);

}
