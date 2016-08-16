/* v4l2n: V4L2 CLI */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <limits.h>
#include <sys/wait.h>
#include <stdint.h>
#include <setjmp.h>

#include "ion.h"
#include "linux/videodev2.h"
#include "linux/v4l2-subdev.h"

#define FALSE		0
#define TRUE		(!FALSE)

#define STRINGIFY_(x)	#x
#define STRINGIFY(x)	STRINGIFY_(x)

#define MIN(a,b)	((a) <= (b) ? (a) : (b))
#define MAX(a,b)	((a) >= (b) ? (a) : (b))
#define CLEAR(x)	memset(&(x), 0, sizeof(x));
#define SIZE(x)		(sizeof(x)/sizeof((x)[0]))
#define BIT(x)		(1<<(x))
static unsigned long int _PAGE_SIZE;
static unsigned long int _PAGE_MASK;
#define PAGE_ALIGN(x)	((typeof(x))(((unsigned long int)(x) + _PAGE_SIZE - 1) & _PAGE_MASK))

#define MAX_RING_BUFFERS	20
#define MAX_CAPTURE_BUFFERS	100
#define MAX_BUFFER_SIZE		(64*1024*1024)
#define MAX_PIPES		6

typedef unsigned char bool;

static char *name = "v4l2_demo";

/* Holds information returned by QUERYBUF and needed
 * for subsequent QBUF/DQBUF. Buffers are reused for long sequences. */
struct ring_buffer {
	struct v4l2_buffer querybuf;
	bool queued;
	void *malloc_p;		/* Points to address returned by malloc() */
	void *mmap_p;		/* Points to address returned by mmap() */
	void *ion_buffer_fd;	/* file descriptor of memory allocated via ION */
	void *start;		/* Points to beginning of data in the buffer */
};

/* Used for saving each captured frame if saving was requested */
struct capture_buffer {
	struct v4l2_pix_format pix_format;
	void *image;
	int length;		/* Length of data in the buffer in bytes */
};

struct pipe {
	int fd;
	char *output;
	struct v4l2_format format;
	struct v4l2_requestbuffers reqbufs;
	int num_capture_buffers;
	struct capture_buffer capture_buffers[MAX_CAPTURE_BUFFERS];
	struct ring_buffer ring_buffers[MAX_RING_BUFFERS];
	bool streaming;
	bool active;
	bool msg_full_printed;
};

static struct {
	int verbosity;
	bool save_images;
	bool calculate_stats;
	struct timeval start_time;
	jmp_buf exception;
	unsigned int pipe;
	struct pipe pipes[MAX_PIPES];
} vars;

/* ION memory management.*/
static struct {
	int fd;
	int map_fd[MAX_RING_BUFFERS];
	ion_user_handle_t user_handle[MAX_RING_BUFFERS];
} ion;

struct symbol_list {
	int id;
	const char *symbol;
};
#define SYMBOL_END	{ -1, NULL }

struct token_list {
	int id;
	int flags;
	const char *token;
	const struct symbol_list *symbols;
};
#define TOKEN_END	{ 0, 0, NULL, NULL }
#define TOKEN_F_OPTARG	BIT(0)	/* Token may have optional argument */
#define TOKEN_F_ARG	BIT(1)	/* Token must have argument or it is an error */
#define TOKEN_F_ARG2	BIT(2)	/* Token may have 2 integer arguments */
#define TOKEN_F_ARG4	BIT(3)	/* Token may have 4 integer arguments */

#define V4L2_CID	"V4L2_CID_"
#define CONTROL(id)	{ V4L2_CID_##id, (#id) }
static const struct symbol_list controls[] = {
	CONTROL(BASE),
	CONTROL(USER_BASE),
	CONTROL(USER_CLASS),
	CONTROL(BRIGHTNESS),
	CONTROL(CONTRAST),
	CONTROL(SATURATION),
	CONTROL(HUE),
	CONTROL(AUDIO_VOLUME),
	CONTROL(AUDIO_BALANCE),
	CONTROL(AUDIO_BASS),
	CONTROL(AUDIO_TREBLE),
	CONTROL(AUDIO_MUTE),
	CONTROL(AUDIO_LOUDNESS),
	CONTROL(BLACK_LEVEL),
	CONTROL(AUTO_WHITE_BALANCE),
	CONTROL(DO_WHITE_BALANCE),
	CONTROL(RED_BALANCE),
	CONTROL(BLUE_BALANCE),
	CONTROL(GAMMA),
	CONTROL(WHITENESS),
	CONTROL(EXPOSURE),
	CONTROL(AUTOGAIN),
	CONTROL(GAIN),
	CONTROL(HFLIP),
	CONTROL(VFLIP),
	CONTROL(POWER_LINE_FREQUENCY),
	CONTROL(HUE_AUTO),
	CONTROL(WHITE_BALANCE_TEMPERATURE),
	CONTROL(SHARPNESS),
	CONTROL(BACKLIGHT_COMPENSATION),
	CONTROL(CHROMA_AGC),
	CONTROL(COLOR_KILLER),
	CONTROL(COLORFX),
	CONTROL(AUTOBRIGHTNESS),
	CONTROL(BAND_STOP_FILTER),
	CONTROL(ROTATE),
	CONTROL(BG_COLOR),
	CONTROL(CHROMA_GAIN),
	CONTROL(ILLUMINATORS_1),
	CONTROL(ILLUMINATORS_2),
	CONTROL(MIN_BUFFERS_FOR_CAPTURE),
	CONTROL(MIN_BUFFERS_FOR_OUTPUT),
	CONTROL(ALPHA_COMPONENT),
	CONTROL(COLORFX_CBCR),
	CONTROL(LASTP1),
	CONTROL(USER_MEYE_BASE),
	CONTROL(USER_BTTV_BASE),
	CONTROL(USER_S2255_BASE),
	CONTROL(USER_SI476X_BASE),
	CONTROL(MPEG_BASE),
	CONTROL(MPEG_CLASS),
	CONTROL(MPEG_STREAM_TYPE),
	CONTROL(MPEG_STREAM_PID_PMT),
	CONTROL(MPEG_STREAM_PID_AUDIO),
	CONTROL(MPEG_STREAM_PID_VIDEO),
	CONTROL(MPEG_STREAM_PID_PCR),
	CONTROL(MPEG_STREAM_PES_ID_AUDIO),
	CONTROL(MPEG_STREAM_PES_ID_VIDEO),
	CONTROL(MPEG_STREAM_VBI_FMT),
	CONTROL(MPEG_AUDIO_SAMPLING_FREQ),
	CONTROL(MPEG_AUDIO_ENCODING),
	CONTROL(MPEG_AUDIO_L1_BITRATE),
	CONTROL(MPEG_AUDIO_L2_BITRATE),
	CONTROL(MPEG_AUDIO_L3_BITRATE),
	CONTROL(MPEG_AUDIO_MODE),
	CONTROL(MPEG_AUDIO_MODE_EXTENSION),
	CONTROL(MPEG_AUDIO_EMPHASIS),
	CONTROL(MPEG_AUDIO_CRC),
	CONTROL(MPEG_AUDIO_MUTE),
	CONTROL(MPEG_AUDIO_AAC_BITRATE),
	CONTROL(MPEG_AUDIO_AC3_BITRATE),
	CONTROL(MPEG_AUDIO_DEC_PLAYBACK),
	CONTROL(MPEG_AUDIO_DEC_MULTILINGUAL_PLAYBACK),
	CONTROL(MPEG_VIDEO_ENCODING),
	CONTROL(MPEG_VIDEO_ASPECT),
	CONTROL(MPEG_VIDEO_B_FRAMES),
	CONTROL(MPEG_VIDEO_GOP_SIZE),
	CONTROL(MPEG_VIDEO_GOP_CLOSURE),
	CONTROL(MPEG_VIDEO_PULLDOWN),
	CONTROL(MPEG_VIDEO_BITRATE_MODE),
	CONTROL(MPEG_VIDEO_BITRATE),
	CONTROL(MPEG_VIDEO_BITRATE_PEAK),
	CONTROL(MPEG_VIDEO_TEMPORAL_DECIMATION),
	CONTROL(MPEG_VIDEO_MUTE),
	CONTROL(MPEG_VIDEO_MUTE_YUV),
	CONTROL(MPEG_VIDEO_DECODER_SLICE_INTERFACE),
	CONTROL(MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER),
	CONTROL(MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB),
	CONTROL(MPEG_VIDEO_FRAME_RC_ENABLE),
	CONTROL(MPEG_VIDEO_HEADER_MODE),
	CONTROL(MPEG_VIDEO_MAX_REF_PIC),
	CONTROL(MPEG_VIDEO_MB_RC_ENABLE),
	CONTROL(MPEG_VIDEO_MULTI_SLICE_MAX_BYTES),
	CONTROL(MPEG_VIDEO_MULTI_SLICE_MAX_MB),
	CONTROL(MPEG_VIDEO_MULTI_SLICE_MODE),
	CONTROL(MPEG_VIDEO_VBV_SIZE),
	CONTROL(MPEG_VIDEO_DEC_PTS),
	CONTROL(MPEG_VIDEO_DEC_FRAME),
	CONTROL(MPEG_VIDEO_VBV_DELAY),
	CONTROL(MPEG_VIDEO_REPEAT_SEQ_HEADER),
	CONTROL(MPEG_VIDEO_H263_I_FRAME_QP),
	CONTROL(MPEG_VIDEO_H263_P_FRAME_QP),
	CONTROL(MPEG_VIDEO_H263_B_FRAME_QP),
	CONTROL(MPEG_VIDEO_H263_MIN_QP),
	CONTROL(MPEG_VIDEO_H263_MAX_QP),
	CONTROL(MPEG_VIDEO_H264_I_FRAME_QP),
	CONTROL(MPEG_VIDEO_H264_P_FRAME_QP),
	CONTROL(MPEG_VIDEO_H264_B_FRAME_QP),
	CONTROL(MPEG_VIDEO_H264_MIN_QP),
	CONTROL(MPEG_VIDEO_H264_MAX_QP),
	CONTROL(MPEG_VIDEO_H264_8X8_TRANSFORM),
	CONTROL(MPEG_VIDEO_H264_CPB_SIZE),
	CONTROL(MPEG_VIDEO_H264_ENTROPY_MODE),
	CONTROL(MPEG_VIDEO_H264_I_PERIOD),
	CONTROL(MPEG_VIDEO_H264_LEVEL),
	CONTROL(MPEG_VIDEO_H264_LOOP_FILTER_ALPHA),
	CONTROL(MPEG_VIDEO_H264_LOOP_FILTER_BETA),
	CONTROL(MPEG_VIDEO_H264_LOOP_FILTER_MODE),
	CONTROL(MPEG_VIDEO_H264_PROFILE),
	CONTROL(MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT),
	CONTROL(MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH),
	CONTROL(MPEG_VIDEO_H264_VUI_SAR_ENABLE),
	CONTROL(MPEG_VIDEO_H264_VUI_SAR_IDC),
	CONTROL(MPEG_VIDEO_H264_SEI_FRAME_PACKING),
	CONTROL(MPEG_VIDEO_H264_SEI_FP_CURRENT_FRAME_0),
	CONTROL(MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE),
	CONTROL(MPEG_VIDEO_H264_FMO),
	CONTROL(MPEG_VIDEO_H264_FMO_MAP_TYPE),
	CONTROL(MPEG_VIDEO_H264_FMO_SLICE_GROUP),
	CONTROL(MPEG_VIDEO_H264_FMO_CHANGE_DIRECTION),
	CONTROL(MPEG_VIDEO_H264_FMO_CHANGE_RATE),
	CONTROL(MPEG_VIDEO_H264_FMO_RUN_LENGTH),
	CONTROL(MPEG_VIDEO_H264_ASO),
	CONTROL(MPEG_VIDEO_H264_ASO_SLICE_ORDER),
	CONTROL(MPEG_VIDEO_H264_HIERARCHICAL_CODING),
	CONTROL(MPEG_VIDEO_H264_HIERARCHICAL_CODING_TYPE),
	CONTROL(MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER),
	CONTROL(MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_QP),
	CONTROL(MPEG_VIDEO_MPEG4_I_FRAME_QP),
	CONTROL(MPEG_VIDEO_MPEG4_P_FRAME_QP),
	CONTROL(MPEG_VIDEO_MPEG4_B_FRAME_QP),
	CONTROL(MPEG_VIDEO_MPEG4_MIN_QP),
	CONTROL(MPEG_VIDEO_MPEG4_MAX_QP),
	CONTROL(MPEG_VIDEO_MPEG4_LEVEL),
	CONTROL(MPEG_VIDEO_MPEG4_PROFILE),
	CONTROL(MPEG_VIDEO_MPEG4_QPEL),
	CONTROL(MPEG_CX2341X_BASE),
	CONTROL(MPEG_CX2341X_VIDEO_SPATIAL_FILTER_MODE),
	CONTROL(MPEG_CX2341X_VIDEO_SPATIAL_FILTER),
	CONTROL(MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE),
	CONTROL(MPEG_CX2341X_VIDEO_CHROMA_SPATIAL_FILTER_TYPE),
	CONTROL(MPEG_CX2341X_VIDEO_TEMPORAL_FILTER_MODE),
	CONTROL(MPEG_CX2341X_VIDEO_TEMPORAL_FILTER),
	CONTROL(MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE),
	CONTROL(MPEG_CX2341X_VIDEO_LUMA_MEDIAN_FILTER_BOTTOM),
	CONTROL(MPEG_CX2341X_VIDEO_LUMA_MEDIAN_FILTER_TOP),
	CONTROL(MPEG_CX2341X_VIDEO_CHROMA_MEDIAN_FILTER_BOTTOM),
	CONTROL(MPEG_CX2341X_VIDEO_CHROMA_MEDIAN_FILTER_TOP),
	CONTROL(MPEG_CX2341X_STREAM_INSERT_NAV_PACKETS),
	CONTROL(MPEG_MFC51_BASE),
	CONTROL(MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY),
	CONTROL(MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE),
	CONTROL(MPEG_MFC51_VIDEO_FRAME_SKIP_MODE),
	CONTROL(MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE),
	CONTROL(MPEG_MFC51_VIDEO_PADDING),
	CONTROL(MPEG_MFC51_VIDEO_PADDING_YUV),
	CONTROL(MPEG_MFC51_VIDEO_RC_FIXED_TARGET_BIT),
	CONTROL(MPEG_MFC51_VIDEO_RC_REACTION_COEFF),
	CONTROL(MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_ACTIVITY),
	CONTROL(MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_DARK),
	CONTROL(MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_SMOOTH),
	CONTROL(MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_STATIC),
	CONTROL(MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P),
	CONTROL(CAMERA_CLASS_BASE),
	CONTROL(CAMERA_CLASS),
	CONTROL(EXPOSURE_AUTO),
	CONTROL(EXPOSURE_ABSOLUTE),
	CONTROL(EXPOSURE_AUTO_PRIORITY),
	CONTROL(PAN_RELATIVE),
	CONTROL(TILT_RELATIVE),
	CONTROL(PAN_RESET),
	CONTROL(TILT_RESET),
	CONTROL(PAN_ABSOLUTE),
	CONTROL(TILT_ABSOLUTE),
	CONTROL(FOCUS_ABSOLUTE),
	CONTROL(FOCUS_RELATIVE),
	CONTROL(FOCUS_AUTO),
	CONTROL(ZOOM_ABSOLUTE),
	CONTROL(ZOOM_RELATIVE),
	CONTROL(ZOOM_CONTINUOUS),
	CONTROL(PRIVACY),
	CONTROL(IRIS_ABSOLUTE),
	CONTROL(IRIS_RELATIVE),
	CONTROL(AUTO_EXPOSURE_BIAS),
	CONTROL(AUTO_N_PRESET_WHITE_BALANCE),
	CONTROL(WIDE_DYNAMIC_RANGE),
	CONTROL(IMAGE_STABILIZATION),
	CONTROL(ISO_SENSITIVITY),
	CONTROL(ISO_SENSITIVITY_AUTO),
	CONTROL(EXPOSURE_METERING),
	CONTROL(SCENE_MODE),
	CONTROL(3A_LOCK),
	CONTROL(AUTO_FOCUS_START),
	CONTROL(AUTO_FOCUS_STOP),
	CONTROL(AUTO_FOCUS_STATUS),
	CONTROL(AUTO_FOCUS_RANGE),
	CONTROL(FM_TX_CLASS_BASE),
	CONTROL(FM_TX_CLASS),
	CONTROL(RDS_TX_DEVIATION),
	CONTROL(RDS_TX_PI),
	CONTROL(RDS_TX_PTY),
	CONTROL(RDS_TX_PS_NAME),
	CONTROL(RDS_TX_RADIO_TEXT),
	CONTROL(AUDIO_LIMITER_ENABLED),
	CONTROL(AUDIO_LIMITER_RELEASE_TIME),
	CONTROL(AUDIO_LIMITER_DEVIATION),
	CONTROL(AUDIO_COMPRESSION_ENABLED),
	CONTROL(AUDIO_COMPRESSION_GAIN),
	CONTROL(AUDIO_COMPRESSION_THRESHOLD),
	CONTROL(AUDIO_COMPRESSION_ATTACK_TIME),
	CONTROL(AUDIO_COMPRESSION_RELEASE_TIME),
	CONTROL(PILOT_TONE_ENABLED),
	CONTROL(PILOT_TONE_DEVIATION),
	CONTROL(PILOT_TONE_FREQUENCY),
	CONTROL(TUNE_PREEMPHASIS),
	CONTROL(TUNE_POWER_LEVEL),
	CONTROL(TUNE_ANTENNA_CAPACITOR),
	CONTROL(FLASH_CLASS_BASE),
	CONTROL(FLASH_CLASS),
	CONTROL(FLASH_LED_MODE),
	CONTROL(FLASH_STROBE_SOURCE),
	CONTROL(FLASH_STROBE),
	CONTROL(FLASH_STROBE_STOP),
	CONTROL(FLASH_STROBE_STATUS),
	CONTROL(FLASH_TIMEOUT),
	CONTROL(FLASH_INTENSITY),
	CONTROL(FLASH_TORCH_INTENSITY),
	CONTROL(FLASH_INDICATOR_INTENSITY),
	CONTROL(FLASH_FAULT),
	CONTROL(FLASH_CHARGE),
	CONTROL(FLASH_READY),
	CONTROL(JPEG_CLASS_BASE),
	CONTROL(JPEG_CLASS),
	CONTROL(JPEG_CHROMA_SUBSAMPLING),
	CONTROL(JPEG_RESTART_INTERVAL),
	CONTROL(JPEG_COMPRESSION_QUALITY),
	CONTROL(JPEG_ACTIVE_MARKER),
	CONTROL(IMAGE_SOURCE_CLASS_BASE),
	CONTROL(IMAGE_SOURCE_CLASS),
	CONTROL(VBLANK),
	CONTROL(HBLANK),
	CONTROL(ANALOGUE_GAIN),
	CONTROL(IMAGE_PROC_CLASS_BASE),
	CONTROL(IMAGE_PROC_CLASS),
	CONTROL(LINK_FREQ),
	CONTROL(PIXEL_RATE),
	CONTROL(TEST_PATTERN),
	CONTROL(DV_CLASS_BASE),
	CONTROL(DV_CLASS),
	CONTROL(DV_TX_HOTPLUG),
	CONTROL(DV_TX_RXSENSE),
	CONTROL(DV_TX_EDID_PRESENT),
	CONTROL(DV_TX_MODE),
	CONTROL(DV_TX_RGB_RANGE),
	CONTROL(DV_RX_POWER_PRESENT),
	CONTROL(DV_RX_RGB_RANGE),
	CONTROL(FM_RX_CLASS_BASE),
	CONTROL(FM_RX_CLASS),
	CONTROL(TUNE_DEEMPHASIS),
	CONTROL(RDS_RECEPTION),
	SYMBOL_END
};

#define V4L2_BUF_TYPE	"V4L2_BUF_TYPE_"
#define BUFTYPE(id)	{ V4L2_BUF_TYPE_##id, (#id) }
static const struct symbol_list v4l2_buf_types[] = {
	BUFTYPE(VIDEO_CAPTURE),
	BUFTYPE(VIDEO_OUTPUT),
	BUFTYPE(VIDEO_OVERLAY),
	BUFTYPE(VBI_CAPTURE),
	BUFTYPE(VBI_OUTPUT),
	BUFTYPE(SLICED_VBI_CAPTURE),
	BUFTYPE(SLICED_VBI_OUTPUT),
	BUFTYPE(VIDEO_OUTPUT_OVERLAY),
	BUFTYPE(PRIVATE),
	SYMBOL_END
};

#define V4L2_PIX_FMT	"V4L2_PIX_FMT_"
#define PIXFMT(id)	{ V4L2_PIX_FMT_##id, (#id) }
static const struct symbol_list pixelformats[] = {
	PIXFMT(RGB332),
	PIXFMT(RGB444),
	PIXFMT(RGB555),
	PIXFMT(RGB565),
	PIXFMT(RGB555X),
	PIXFMT(RGB565X),
	PIXFMT(BGR666),
	PIXFMT(BGR24),
	PIXFMT(RGB24),
	PIXFMT(BGR32),
	PIXFMT(RGB32),
	PIXFMT(GREY),
	PIXFMT(Y4),
	PIXFMT(Y6),
	PIXFMT(Y10),
	PIXFMT(Y12),
	PIXFMT(Y16),
	PIXFMT(Y10BPACK),
	PIXFMT(PAL8),
	PIXFMT(UV8),
	PIXFMT(YVU410),
	PIXFMT(YVU420),
	PIXFMT(YUYV),
	PIXFMT(YYUV),
	PIXFMT(YVYU),
	PIXFMT(UYVY),
	PIXFMT(VYUY),
	PIXFMT(YUV422P),
	PIXFMT(YUV411P),
	PIXFMT(Y41P),
	PIXFMT(YUV444),
	PIXFMT(YUV555),
	PIXFMT(YUV565),
	PIXFMT(YUV32),
	PIXFMT(YUV410),
	PIXFMT(YUV420),
	PIXFMT(HI240),
	PIXFMT(HM12),
	PIXFMT(M420),
	PIXFMT(NV12),
	PIXFMT(NV21),
	PIXFMT(NV16),
	PIXFMT(NV61),
	PIXFMT(NV24),
	PIXFMT(NV42),
	PIXFMT(NV12M),
	PIXFMT(NV21M),
	PIXFMT(NV12MT),
	PIXFMT(NV12MT_16X16),
	PIXFMT(YUV420M),
	PIXFMT(YVU420M),
	PIXFMT(SBGGR8),
	PIXFMT(SGBRG8),
	PIXFMT(SGRBG8),
	PIXFMT(SRGGB8),
	PIXFMT(SBGGR10),
	PIXFMT(SGBRG10),
	PIXFMT(SGRBG10),
	PIXFMT(SRGGB10),
	PIXFMT(SBGGR12),
	PIXFMT(SGBRG12),
	PIXFMT(SGRBG12),
	PIXFMT(SRGGB12),
	PIXFMT(SBGGR10ALAW8),
	PIXFMT(SGBRG10ALAW8),
	PIXFMT(SGRBG10ALAW8),
	PIXFMT(SRGGB10ALAW8),
	PIXFMT(SBGGR10DPCM8),
	PIXFMT(SGBRG10DPCM8),
	PIXFMT(SGRBG10DPCM8),
	PIXFMT(SRGGB10DPCM8),
	PIXFMT(SBGGR16),
	PIXFMT(MJPEG),
	PIXFMT(JPEG),
	PIXFMT(DV),
	PIXFMT(MPEG),
	PIXFMT(H264),
	PIXFMT(H264_NO_SC),
	PIXFMT(H264_MVC),
	PIXFMT(H263),
	PIXFMT(MPEG1),
	PIXFMT(MPEG2),
	PIXFMT(MPEG4),
	PIXFMT(XVID),
	PIXFMT(VC1_ANNEX_G),
	PIXFMT(VC1_ANNEX_L),
	PIXFMT(VP8),
	PIXFMT(CPIA1),
	PIXFMT(WNVA),
	PIXFMT(SN9C10X),
	PIXFMT(SN9C20X_I420),
	PIXFMT(PWC1),
	PIXFMT(PWC2),
	PIXFMT(ET61X251),
	PIXFMT(SPCA501),
	PIXFMT(SPCA505),
	PIXFMT(SPCA508),
	PIXFMT(SPCA561),
	PIXFMT(PAC207),
	PIXFMT(MR97310A),
	PIXFMT(JL2005BCD),
	PIXFMT(SN9C2028),
	PIXFMT(SQ905C),
	PIXFMT(PJPG),
	PIXFMT(OV511),
	PIXFMT(OV518),
	PIXFMT(STV0680),
	PIXFMT(TM6000),
	PIXFMT(CIT_YYVYUY),
	PIXFMT(KONICA420),
	PIXFMT(JPGL),
	PIXFMT(SE401),
	PIXFMT(S5C_UYVY_JPG),
	SYMBOL_END
};

static const struct symbol_list v4l2_memory[] = {
	{V4L2_MEMORY_MMAP, "MMAP"},
	{V4L2_MEMORY_USERPTR, "USERPTR"},
	SYMBOL_END
};

static void print(int lvl, char *msg, ...)
{
	va_list ap;

	if (vars.verbosity < lvl)
		return;

	va_start(ap, msg);
	vprintf(msg, ap);
	va_end(ap);
	fflush(stdout);
}

static void error(char *msg, ...)
{
	FILE *f = stdout;
	va_list ap;
	int e = errno;

	va_start(ap, msg);
	fprintf(f, "%s: ", name);
	vfprintf(f, msg, ap);
	if (e)
		fprintf(f, ": %s (%i)", strerror(e), e);
	fprintf(f, "\n");
	va_end(ap);
	longjmp(vars.exception, 1);
	exit(1);
}

#define itd_xioctl(io, arg) itd_xioctl_(#io, io, arg)

static void itd_xioctl_(char *ios, int ion, void *arg)
{
	int fd = vars.pipes[vars.pipe].fd;
	int r = ioctl(fd, ion, arg);
	if (r)
		error("%s failed on fd %i", ios, fd);
}

static int itd_xioctl_try(int ion, void *arg)
{
	int r = ioctl(vars.pipes[vars.pipe].fd, ion, arg);
	if (r != 0) {
		int e = -errno;
		if (e == 0)
			return INT_MIN;
		return e;
	}
	return 0;
}

static void *ralloc(void *p, int s)
{
	void *r = realloc(p, s);
	if (r)
		return r;
	free(p);
	error("memory reallocation failed");
	return NULL;
}

static void symbol_dump(const char *prefix, const struct symbol_list *list)
{
	int i;
	for (i = 0; list[i].symbol != NULL; i++)
		print(0, "%s%s [0x%08X]\n", prefix, list[i].symbol, list[i].id);
}

static int symbol_get(const struct symbol_list *list, const char **symbol)
{
	const char *start;
	const char *end;
	int r, i;

	if (!symbol || !symbol[0])
		error("symbol missing");

	start = *symbol;

	if (isdigit(start[0])) {
		r = strtol(start, (char **)&end, 0);
		if (end == start)
			error("zero-length symbol value");
	} else {
		end = start;
		while (isalnum(*end) || *end == '_')
			end++;
		if (start == end)
			error("zero-length symbol");
		if (!list)
			error("only numeric value allowed");
		for (i = 0; list[i].symbol != NULL; i++) {
			if (strncasecmp(start, list[i].symbol, end - start) ==
			    0)
				break;
		}
		if (list[i].symbol == NULL)
			error("symbol `%s' not found", start);
		r = list[i].id;
	}

	*symbol = end;
	return r;
}

static int values_get(int *val, int val_size, const char **ptr)
{
	const char *s = *ptr;
	bool paren = FALSE;
	int i = 0;

	if (!s)
		error("missing integer values");

	if (*s == '(') {
		s++;
		paren = TRUE;
	}

	while (i < val_size) {
		char *a;
		val[i] = strtol(s, &a, 0);
		if (a == s)
			break;
		s = a;
		i++;
		if (*s != ',' && *s != '/' && *s != ';')
			break;
		s++;
	}

	if (paren) {
		if (*s != ')')
			error("missing closing parenthesis");
		s++;
	}

	*ptr = s;
	return i;
}

static void value_get(int *val, int val_size, const char **ptr)
{
	int m = values_get(val, val_size, ptr);
	if (m != val_size)
		error("too little arguments, %i given but %i expected", m,
		      val_size);
}

static int token_get(const struct token_list *list, const char **token,
		     int val[4])
{
	const char *start;
	const char *end;
	int r, i;

	if (!token || !token[0])
		error("token missing");

	CLEAR(*val);
	end = start = *token;
	while (isalpha(*end) || *end == '_' || *end == '.')
		end++;
	if (start == end)
		error("zero-length token");
	for (i = 0; list[i].token != NULL; i++) {
		if (strncasecmp(start, list[i].token, end - start) == 0)
			break;
	}
	if (list[i].token == NULL)
		error("unrecognized token `%s'", start);
	r = list[i].id;

	if (*end == '=' || *end == ':') {
		if (!(list[i].flags & (TOKEN_F_ARG | TOKEN_F_OPTARG)))
			error("token has argument but should not have");
		end++;
		if (list[i].symbols) {
			val[0] = symbol_get(list[i].symbols, &end);
		} else if (list[i].flags & TOKEN_F_ARG4) {
			value_get(val, 4, &end);
		} else if (list[i].flags & TOKEN_F_ARG2) {
			value_get(val, 2, &end);
		} else {
			value_get(val, 1, &end);
		}
	} else {
		if (list[i].flags & TOKEN_F_ARG)
			error
			    ("token does not have an argument but should have");
	}

	/* Go to the beginning of next token in the list */
	if (*end && *end == ',')
		end++;
	*token = end;
	return r;
}

static const char *symbol_str(int id, const struct symbol_list list[])
{
	static char buffer[200];
	int i;

	for (i = 0; list[i].symbol; i++)
		if (list[i].id == id)
			break;

	if (list[i].symbol) {
		if (id < 1000)
			sprintf(buffer, "%s [%i]", list[i].symbol, id);
		else
			sprintf(buffer, "%s [0x%08X]", list[i].symbol, id);
	} else {
		sprintf(buffer, "%i", id);
	}
	return buffer;
}

static const char *symbol_flag_str(int id, const struct symbol_list list[])
{
	const int MARGIN = 32;
	static char buffer[512];
	int len = 0;
	int i;

	for (i = 0; list[i].symbol; i++) {
		if ((id & list[i].id) == list[i].id) {
			if (len == 0) {
				strcpy(buffer, list[i].symbol);
			} else {
				strcat(buffer, " ");
				strcat(buffer, list[i].symbol);
				len++;
			}
			len += strlen(list[i].symbol);
			if (len > sizeof(buffer) - MARGIN)
				error("buffer outrun while printing flags");
		}
	}

	if (len == 0) {
		len += sprintf(buffer, "0x%08X", id);
	} else {
		len += sprintf(&buffer[len], " [0x%08X]", id);
	}
	return buffer;
}

static void print_time(void)
{
	struct timeval tv;

	if (gettimeofday(&tv, NULL) < 0)
		error("gettimeofday failed");
	tv.tv_sec -= vars.start_time.tv_sec;
	tv.tv_usec -= vars.start_time.tv_usec;
	while (tv.tv_usec < 0) {
		tv.tv_usec += 1000000;
		tv.tv_sec -= 1;
	}
	print(1, "[%3i.%06i]\n", tv.tv_sec, tv.tv_usec);
}

static void write_file(const char *name, const char *data, int size)
{
}

static void itr_iterate(void (*itd) (const char *), const char *arg)
{
	for (vars.pipe = 0; vars.pipe < MAX_PIPES; vars.pipe++) {
		if (!vars.pipes[vars.pipe].active)
			continue;
		itd(arg);
	}
}

static void itd_vidioc_enuminput(const char *unused)
{
	static const struct symbol_list type[] = {
		{V4L2_INPUT_TYPE_TUNER, "V4L2_INPUT_TYPE_TUNER"},
		{V4L2_INPUT_TYPE_CAMERA, "V4L2_INPUT_TYPE_CAMERA"},
		SYMBOL_END
	};

	struct v4l2_input p;
	int r, i = 0;

	do {
		CLEAR(p);
		p.index = i++;
		print(1, "VIDIOC_ENUMINPUT (index=%i)\n", p.index);
		r = itd_xioctl_try(VIDIOC_ENUMINPUT, &p);
		if (r == 0) {
			print(2, "> index:        %i\n", p.index);
			print(2, "> name:         `%.32s'\n", p.name);
			print(2, "> type:         %s\n",
			      symbol_str(p.type, type));
			print(2, "> audioset:     %i\n", p.audioset);
			print(2, "> tuner:        %i\n", p.tuner);
			print(2, "> std:          %li\n", p.std);
			print(2, "> status:       0x%08X\n", p.status);
			print(2, "> capabilities: 0x%08X\n", p.capabilities);
			print(2, "> reserved[0]:  0x%08X\n", p.reserved[0]);
			print(2, "> reserved[1]:  0x%08X\n", p.reserved[1]);
			print(2, "> reserved[2]:  0x%08X\n", p.reserved[2]);
		}
	} while (r == 0);
	if (r != -EINVAL)
		error("VIDIOC_ENUMINPUT failed");
}

static void itd_streamon(const char *arg)
{
	bool on = (arg != NULL);
	enum v4l2_buf_type t = vars.pipes[vars.pipe].reqbufs.type;
	print(1, "VIDIOC_STREAM%s (type=%s)\n", on ? "ON" : "OFF",
	      symbol_str(t, v4l2_buf_types));
	if (vars.pipes[vars.pipe].streaming == on)
		print(0, "warning: streaming is already in this state\n");
	if (on)
		itd_xioctl(VIDIOC_STREAMON, &t);
	else
		itd_xioctl(VIDIOC_STREAMOFF, &t);
	vars.pipes[vars.pipe].streaming = on;
}

static void itd_vidioc_parm(const char *s)
{
	static const struct symbol_list capturemode[] = {
		{V4L2_MODE_HIGHQUALITY, "V4L2_MODE_HIGHQUALITY"},
		SYMBOL_END
	};
	static const struct symbol_list capability[] = {
		{V4L2_CAP_TIMEPERFRAME, "V4L2_CAP_TIMEPERFRAME"},
		SYMBOL_END
	};
	static const struct token_list list[] = {
		{'t', TOKEN_F_ARG, "type", v4l2_buf_types},
		{'b', TOKEN_F_ARG, "capability", capability},
		{'c', TOKEN_F_ARG, "capturemode", capturemode},
		{'f', TOKEN_F_ARG | TOKEN_F_ARG2, "timeperframe", NULL},
		{'e', TOKEN_F_ARG, "extendedmode", NULL},
		{'r', TOKEN_F_ARG, "readbuffers", NULL},
		{'o', TOKEN_F_ARG, "outputmode", capturemode},
		TOKEN_END
	};
	struct v4l2_streamparm p;

	CLEAR(p);
	p.type = vars.pipes[vars.pipe].reqbufs.type;

	while (*s && *s != '?') {
		int val[4];
		switch (token_get(list, &s, val)) {
		case 't':
			p.type = val[0];
			break;
		case 'b':
			if (p.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
				p.parm.capture.capability = val[0];
			else
				p.parm.output.capability = val[0];
			break;
		case 'c':
			p.parm.capture.capturemode = val[0];
			break;
		case 'f':
			if (p.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				p.parm.capture.timeperframe.numerator = val[0];
				p.parm.capture.timeperframe.denominator =
				    val[1];
			} else {
				p.parm.output.timeperframe.numerator = val[0];
				p.parm.output.timeperframe.denominator = val[1];
			}
			break;
		case 'e':
			p.parm.capture.extendedmode = val[0];
			break;
		case 'r':
			p.parm.capture.readbuffers = val[0];
			break;
		case 'o':
			p.parm.output.outputmode = val[0];
			break;
		}
	}

	if (*s == '?') {
		print(1, "VIDIOC_G_PARM\n");
		itd_xioctl(VIDIOC_G_PARM, &p);
	} else {
		print(1, "VIDIOC_S_PARM\n");
	}

	print(2, ": type:          %s\n", symbol_str(p.type, v4l2_buf_types));
	if (p.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		print(2, ": capability:    %s\n",
		      symbol_str(p.parm.capture.capability, capability));
		print(2, ": capturemode:   %s\n",
		      symbol_str(p.parm.capture.capturemode, capturemode));
		print(2, ": timeperframe:  %i/%i\n",
		      p.parm.capture.timeperframe.numerator,
		      p.parm.capture.timeperframe.denominator);
		print(2, ": extendedmode:  %i\n", p.parm.capture.extendedmode);
		print(2, ": readbuffers:   %i\n", p.parm.capture.readbuffers);
	} else if (p.type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		print(2, ": capability:    %s\n",
		      symbol_str(p.parm.output.capability, capability));
		print(2, ": outputmode:    %s\n",
		      symbol_str(p.parm.capture.capturemode, capturemode));
		print(2, ": timeperframe:  %i/%i\n",
		      p.parm.capture.timeperframe.numerator,
		      p.parm.capture.timeperframe.denominator);
	}

	if (*s != '?') {
		itd_xioctl(VIDIOC_S_PARM, &p);
	}
}

static void print_v4l2_format(int v, struct v4l2_format *f, char c)
{
	print(v, "%c type:          %s\n", c,
	      symbol_str(f->type, v4l2_buf_types));
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		print(v, "%c width:         %i\n", c, f->fmt.pix.width);
		print(v, "%c height:        %i\n", c, f->fmt.pix.height);
		print(v, "%c pixelformat:   %s\n", c,
		      symbol_str(f->fmt.pix.pixelformat, pixelformats));
		print(v, "%c field:         %i\n", c, f->fmt.pix.field);
		print(v, "%c bytesperline:  %i\n", c, f->fmt.pix.bytesperline);
		print(v, "%c sizeimage:     %i\n", c, f->fmt.pix.sizeimage);
		print(v, "%c colorspace:    %i\n", c, f->fmt.pix.colorspace);
		print(v, "%c priv:          %i\n", c, f->fmt.pix.priv);
	}
}

static void itd_vidioc_fmt(bool try, const char *s)
{
	static const struct token_list list[] = {
		{'t', TOKEN_F_ARG, "type", v4l2_buf_types},
		{'w', TOKEN_F_ARG, "width", NULL},
		{'h', TOKEN_F_ARG, "height", NULL},
		{'p', TOKEN_F_ARG, "pixelformat", pixelformats},
		{'f', TOKEN_F_ARG, "field", NULL},
		{'b', TOKEN_F_ARG, "bytesperline", NULL},
		{'s', TOKEN_F_ARG, "sizeimage", NULL},
		{'c', TOKEN_F_ARG, "colorspace", NULL},
		{'r', TOKEN_F_ARG, "priv", NULL},
		TOKEN_END
	};
	struct v4l2_format p;

	CLEAR(p);
	p.type = vars.pipes[vars.pipe].reqbufs.type;

	while (*s && *s != '?') {
		int val[4];
		switch (token_get(list, &s, val)) {
		case 't':
			p.type = val[0];
			break;
		case 'w':
			p.fmt.pix.width = val[0];
			break;
		case 'h':
			p.fmt.pix.height = val[0];
			break;
		case 'p':
			p.fmt.pix.pixelformat = val[0];
			break;
		case 'f':
			p.fmt.pix.field = val[0];
			break;
		case 'b':
			p.fmt.pix.bytesperline = val[0];
			break;
		case 's':
			p.fmt.pix.sizeimage = val[0];
			break;
		case 'c':
			p.fmt.pix.colorspace = val[0];
			break;
		case 'r':
			p.fmt.pix.priv = val[0];
			break;
		}
	}

	if (try) {
		print(1, "VIDIOC_TRY_FMT\n");
		itd_xioctl(VIDIOC_TRY_FMT, &p);
	} else if (*s == '?') {
		print(1, "VIDIOC_G_FMT\n");
		itd_xioctl(VIDIOC_G_FMT, &p);
	} else {
		print(1, "VIDIOC_S_FMT\n");
		print_v4l2_format(0, &p, '<');
		itd_xioctl(VIDIOC_S_FMT, &p);
		vars.pipes[vars.pipe].format = p;
	}
	print_v4l2_format(2, &p, '>');
}

static void itd_vidioc_try_fmt(const char *s)
{
	itd_vidioc_fmt(TRUE, s);
}

static void itd_vidioc_sg_fmt(const char *s)
{
	itd_vidioc_fmt(FALSE, s);
}

static void itd_vidioc_reqbufs(const char *s)
{
	static const struct token_list list[] = {
		{'c', TOKEN_F_ARG, "count", NULL},
		{'t', TOKEN_F_ARG, "type", v4l2_buf_types},
		{'m', TOKEN_F_ARG, "memory", v4l2_memory},
		TOKEN_END
	};
	struct v4l2_requestbuffers *p = &vars.pipes[vars.pipe].reqbufs;

	while (*s && *s != '?') {
		int val[4];
		switch (token_get(list, &s, val)) {
		case 'c':
			p->count = val[0];
			break;
		case 't':
			p->type = val[0];
			break;
		case 'm':
			p->memory = val[0];
			break;
		}
	}

	print(1, "VIDIOC_REQBUFS\n");
	print(2, ": count:   %i\n", p->count);
	print(2, ": type:    %s\n", symbol_str(p->type, v4l2_buf_types));
	print(2, ": memory:  %s\n", symbol_str(p->memory, v4l2_memory));
	itd_xioctl(VIDIOC_REQBUFS, p);
	print(2, "> count:   %i\n", p->count);
}

static void print_buffer(struct v4l2_buffer *b, char c)
{
#define V4L2_BUF_FLAG	"V4L2_BUF_FLAG_"
#define BUF_FLAG(id)	{ V4L2_BUF_FLAG_##id, (#id) }

	static const struct symbol_list buf_flags[] = {
		BUF_FLAG(MAPPED),
		BUF_FLAG(QUEUED),
		BUF_FLAG(DONE),
		BUF_FLAG(ERROR),
		BUF_FLAG(KEYFRAME),
		BUF_FLAG(PFRAME),
		BUF_FLAG(BFRAME),
		BUF_FLAG(TIMECODE),
		SYMBOL_END
	};

	const int v = 2;
	print(v, "%c index:     %i\n", c, b->index);
	print(v, "%c type:      %s\n", c, symbol_str(b->type, v4l2_buf_types));
	print(v, "%c bytesused: %i\n", c, b->bytesused);
	print(v, "%c flags:     %s\n", c, symbol_flag_str(b->flags, buf_flags));
	print(v, "%c field:     %i\n", c, b->field);
	print(v, "%c timestamp: %i.%06i\n", c, b->timestamp.tv_sec,
	      b->timestamp.tv_usec);
	print(v, "%c timecode:  type:%i flags:0x%X %02i:%02i:%02i.%i\n", c,
	      b->timecode.type, b->timecode.flags, b->timecode.hours,
	      b->timecode.minutes, b->timecode.seconds, b->timecode.frames);
	print(v, "%c sequence:  %i\n", c, b->sequence);
	print(v, "%c memory:    %s\n", c, symbol_str(b->memory, v4l2_memory));
	if (b->memory == V4L2_MEMORY_MMAP)
		print(v, "%c offset:    0x%08X\n", c, b->m.offset);
	else if (b->memory == V4L2_MEMORY_USERPTR)
		print(v, "%c userptr:   0x%08X\n", c, b->m.userptr);
	print(v, "%c length:    %i\n", c, b->length);
//      print(v, "%c input:     %i\n", c, b->input);
}

static void itd_vidioc_querybuf_cleanup(void)
{
	int i;

	for (i = 0; i < MAX_RING_BUFFERS; i++) {
		struct ring_buffer *rb = &vars.pipes[vars.pipe].ring_buffers[i];
		if (vars.pipe == 0) {	/*ToDo: Proper clean-up. */
			if (ion.fd && ion.user_handle[i]) {
				if (ion_free(ion.fd, ion.user_handle[i]))
					error("ion_free failed");
			}
			if (ion.map_fd[i])
				close(ion.map_fd[i]);
		}
		if (rb->mmap_p) {
			int r = munmap(rb->mmap_p, rb->querybuf.length);
			if (r)
				error("munmap failed");
		}
		CLEAR(*rb);
	}
}

static void itd_vidioc_querybuf(const char *unused)
{
	const enum v4l2_buf_type t = vars.pipes[vars.pipe].reqbufs.type;
	const int bufs = vars.pipes[vars.pipe].reqbufs.count;
	int i;

	itd_vidioc_querybuf_cleanup();

	if (t != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		error("unsupported operation type");

	if (vars.pipes[vars.pipe].reqbufs.memory != V4L2_MEMORY_MMAP &&
	    vars.pipes[vars.pipe].reqbufs.memory != V4L2_MEMORY_USERPTR)
		error("unsupported memory type");

	if (bufs > MAX_RING_BUFFERS)
		error("too many ring buffers");

	for (i = 0; i < bufs; i++) {
		struct ring_buffer *rb = &vars.pipes[vars.pipe].ring_buffers[i];

		CLEAR(rb->querybuf);
		rb->querybuf.type = t;
		rb->querybuf.memory = vars.pipes[vars.pipe].reqbufs.memory;
		rb->querybuf.index = i;
		print(1, "VIDIOC_QUERYBUF index:%i\n", rb->querybuf.index);
		itd_xioctl(VIDIOC_QUERYBUF, &rb->querybuf);
		print_buffer(&rb->querybuf, '>');

		if (rb->querybuf.memory == V4L2_MEMORY_MMAP) {
			void *p = mmap(NULL, rb->querybuf.length,
				       PROT_READ | PROT_WRITE, MAP_SHARED,
				       vars.pipes[vars.pipe].fd,
				       rb->querybuf.m.offset);
			if (p == MAP_FAILED)
				error("mmap failed");
			rb->mmap_p = p;
			rb->start = p;
		} else if (rb->querybuf.memory == V4L2_MEMORY_USERPTR) {
			static const int FILLER = 0xFE;
			int s =
			    PAGE_ALIGN(vars.pipes[vars.pipe].format.fmt.pix.
				       sizeimage) + _PAGE_SIZE - 1;
			unsigned char *p = NULL;

			if (ion_alloc
			    (ion.fd, s, 0, ION_HEAP_TYPE_DMA_MASK, 0,
			     &ion.user_handle[i]) < 0) {
				error("ion_alloc failed");
			}
			ion_map(ion.fd, ion.user_handle[i], s,
				PROT_READ | PROT_WRITE, MAP_SHARED, 0, &p,
				&ion.map_fd[i]);

			if (p == NULL)
				error("malloc failed");
			memset(p, FILLER, s);
			rb->malloc_p = p;
			rb->start = PAGE_ALIGN(p);
		}
	}
}

static void capture_buffer_stats(void *image, struct v4l2_format *format)
{
	static const int BPP = 2;
	static const int NUM = 0;
	static const int SUM = 1;
	static const int MIN = 2;
	static const int MAX = 3;
	long long stat[4][4];
	int y, x, p, stride;
	unsigned char *line;

	if (!vars.calculate_stats)
		return;

	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		error("bad buffer type for statistics");

	switch (format->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		break;
	default:
		error("not supported format for statistics");
	}

	for (p = 0; p < 4; p++) {
		stat[p][NUM] = 0;
		stat[p][SUM] = 0;
		stat[p][MIN] = INT_MAX;
		stat[p][MAX] = 0;
	}

	stride = format->fmt.pix.bytesperline;
	line = image;
	for (y = 0; y < format->fmt.pix.height; y++) {
		unsigned char *ptr = line;
		for (x = 0; x < format->fmt.pix.width; x++) {
			int v = ptr[0] | (ptr[1] << 8);
			p = ((y & 1) << 1) | (x & 1);
			stat[p][NUM]++;
			stat[p][SUM] += v;
			stat[p][MIN] = MIN(stat[p][MIN], v);
			stat[p][MAX] = MAX(stat[p][MAX], v);
			ptr += BPP;
		}
		line += stride;
	}

	for (p = 0; p < 4; p++)
		print(0, "STATISTICS[%i] %.3f %i %i %i\n", p,
		      (double)stat[p][SUM] / stat[p][NUM], (int)stat[p][MIN],
		      (int)stat[p][MAX]);
}

static void itd_capture_buffer_save(void *image, struct v4l2_format *format,
				    struct v4l2_buffer *buffer)
{
	struct capture_buffer *cb;
	unsigned int shift;
	unsigned int mask;

	if (buffer->bytesused < 0 || buffer->bytesused >= MAX_BUFFER_SIZE) {
		print(1, "Bad buffer size %i bytes. Not processing.\n",
		      buffer->bytesused);
		return;
	}

	if (!vars.save_images)
		return;

	if (vars.pipes[vars.pipe].num_capture_buffers >= MAX_CAPTURE_BUFFERS) {
		if (!vars.pipes[vars.pipe].msg_full_printed) {
			print(1, "Buffers full. Not saving the rest\n");
			vars.pipes[vars.pipe].msg_full_printed = TRUE;
		}
		return;
	}

	print(1, "%s\n", __func__);
	cb = &vars.pipes[vars.pipe].capture_buffers[vars.pipes[vars.pipe].
						    num_capture_buffers++];
	cb->pix_format = format->fmt.pix;
	cb->length = buffer->bytesused;
	cb->image = malloc(cb->length);
	if (!cb->image)
		error("out of memory");

	switch (format->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:

	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR10ALAW8:
	case V4L2_PIX_FMT_SGBRG10ALAW8:
	case V4L2_PIX_FMT_SGRBG10ALAW8:
	case V4L2_PIX_FMT_SRGGB10ALAW8:
	case V4L2_PIX_FMT_SBGGR10DPCM8:
	case V4L2_PIX_FMT_SGBRG10DPCM8:
	case V4L2_PIX_FMT_SGRBG10DPCM8:
	case V4L2_PIX_FMT_SRGGB10DPCM8:
	default:
		memcpy(cb->image, image, cb->length);
		break;
	}
}

static void itd_vidioc_dqbuf(void)
{
	enum v4l2_buf_type t = vars.pipes[vars.pipe].reqbufs.type;
	enum v4l2_memory m = vars.pipes[vars.pipe].reqbufs.memory;
	struct v4l2_buffer b;
	int i;

	CLEAR(b);
	b.type = t;
	b.memory = m;
	print(1, "VIDIOC_DQBUF ");
	itd_xioctl(VIDIOC_DQBUF, &b);
	print_time();
	print_buffer(&b, '>');
	i = b.index;
	if (i < 0 || i >= MAX_RING_BUFFERS)
		error("index out of range");

	if (b.bytesused > vars.pipes[vars.pipe].format.fmt.pix.sizeimage)
		error("Bad buffer size %i (sizeimage %i)",
		      b.bytesused,
		      vars.pipes[vars.pipe].format.fmt.pix.sizeimage);
	if (b.bytesused > vars.pipes[vars.pipe].ring_buffers[i].querybuf.length)
		print(1, "warning: Bad buffer size %i (querybuf %i)\n",
		      b.bytesused,
		      vars.pipes[vars.pipe].ring_buffers[i].querybuf.length);

	capture_buffer_stats(vars.pipes[vars.pipe].ring_buffers[i].start,
			     &vars.pipes[vars.pipe].format);
	itd_capture_buffer_save(vars.pipes[vars.pipe].ring_buffers[i].start,
				&vars.pipes[vars.pipe].format, &b);
	vars.pipes[vars.pipe].ring_buffers[i].queued = FALSE;
}

static void itd_vidioc_qbuf(void)
{
	enum v4l2_buf_type t = vars.pipes[vars.pipe].reqbufs.type;
	enum v4l2_memory m = vars.pipes[vars.pipe].reqbufs.memory;
	struct v4l2_buffer b;
	int i;

	for (i = 0; i < vars.pipes[vars.pipe].reqbufs.count; i++)
		if (!vars.pipes[vars.pipe].ring_buffers[i].queued)
			break;

	CLEAR(b);
	b.type = t;
	b.index = i;
	b.memory = m;

	if (m == V4L2_MEMORY_USERPTR) {
		b.m.userptr =
		    (unsigned long)vars.pipes[vars.pipe].ring_buffers[i].start;
		b.length = vars.pipes[vars.pipe].format.fmt.pix.sizeimage;
	} else if (m == V4L2_MEMORY_MMAP) {
		/* Nothing here */
	} else
		error("unsupported capture memory");

	print(1, "VIDIOC_QBUF index:%i\n", i);
	print_buffer(&b, '>');
	itd_xioctl(VIDIOC_QBUF, &b);
	vars.pipes[vars.pipe].ring_buffers[i].queued = TRUE;
}
#include <unistd.h>
static void itr_capture(int frames)
{
	itr_iterate(itd_vidioc_querybuf, NULL);

	if (frames <= 0)
		return;

	for (vars.pipe = 0; vars.pipe < MAX_PIPES; vars.pipe++) {
		int i;
		const int bufs = vars.pipes[vars.pipe].reqbufs.count;
		const int tail = bufs;	//MIN(bufs, frames);
		if (!vars.pipes[vars.pipe].active)
			continue;
		for (i = 0; i < tail; i++)
			itd_vidioc_qbuf();
	}

	itr_iterate(itd_streamon, (char *)TRUE);

	do {
		for (vars.pipe = 0; vars.pipe < MAX_PIPES; vars.pipe++) {
			if (!vars.pipes[vars.pipe].active)
				continue;
			itd_vidioc_dqbuf();
			if (frames > vars.pipes[vars.pipe].reqbufs.count)
				itd_vidioc_qbuf();
		}
	} while (--frames);

	itr_iterate(itd_streamon, (char *)FALSE);
}

static __u32 get_control_id(const char *name)
{
	__u32 id;

	if (isdigit(*name)) {
		int v;
		if (sscanf(name, "%i", &v) != 1)
			error("bad numeric id");
		id = v;
	} else {
		int i;
		for (i = 0; controls[i].symbol != NULL; i++) {
			if (strcmp(name, controls[i].symbol) == 0)
				break;
			if ((strlen(name) >= sizeof(V4L2_CID)) &&
			    (memcmp(name, V4L2_CID, sizeof(V4L2_CID) - 1) == 0)
			    &&
			    (strcmp
			     (name + sizeof(V4L2_CID) - 1,
			      controls[i].symbol) == 0))
				break;
		}
		if (controls[i].symbol == NULL)
			error("unknown control");
		id = controls[i].id;
	}

	return id;
}

static void itd_close_device(const char *unused)
{
	if (vars.pipes[vars.pipe].fd == -1)
		return;

	close(vars.pipes[vars.pipe].fd);
	vars.pipes[vars.pipe].fd = -1;
	print(1, "CLOSED video device\n");
}

static void itd_open_device(const char *device)
{
	static const char DEFAULT_DEV[] = "/dev/video2";
	if (device == NULL && vars.pipes[vars.pipe].fd != -1)
		return;

	itd_close_device(NULL);
	if (!device || device[0] == 0)
		device = DEFAULT_DEV;
	
	vars.pipes[vars.pipe].fd = open(device, O_RDWR | O_CLOEXEC);	//O_RDWR | O_NONBLOCK);
	print(1, "OPEN video device `%s' %d\n", device, vars.pipes[vars.pipe].fd);
	if (vars.pipes[vars.pipe].fd == -1)
		error("failed to open `%s'", device);
}

static void itd_vidioc_querycap(const char *unused)
{
	struct v4l2_capability c;

	CLEAR(c);
	itd_xioctl(VIDIOC_QUERYCAP, &c);
	print(1, "VIDIOC_QUERYCAP\n");
	print(2, "> driver:       `%.16s'\n", c.driver);
	print(2, "> card:         `%.32s'\n", c.card);
	print(2, "> bus_info:     `%.32s'\n", c.bus_info);
	print(2, "> version:      %i.%u.%u\n", c.version >> 16,
	      (c.version >> 8) & 0xFF, c.version & 0xFF);
}

static void itd_vidioc_sg_input(const char *arg)
{
	int i;

	if (strchr(arg, '?')) {
		/* G_INPUT */
		itd_xioctl(VIDIOC_G_INPUT, &i);
		print(1, "VIDIOC_G_INPUT -> %i\n", i);
	} else {
		/* S_INPUT */
		i = atoi(arg);
		print(1, "VIDIOC_S_INPUT <- %i\n", i);
		itd_xioctl(VIDIOC_S_INPUT, &i);
	}
}

static const char *get_control_name(__u32 id)
{
	static char buf[11];
	int i;

	for (i = 0; controls[i].symbol != NULL; i++)
		if (controls[i].id == id)
			return controls[i].symbol;

	sprintf(buf, "0x%08X", id);
	return buf;
}

static void itd_v4l2_s_ctrl(__u32 id, __s32 val)
{
	struct v4l2_control c;

	CLEAR(c);
	c.id = id;
	c.value = val;
	print(1, "VIDIOC_S_CTRL[%s] = %i\n", get_control_name(id), c.value);
	itd_xioctl(VIDIOC_S_CTRL, &c);
}

static __s32 itd_v4l2_g_ctrl(__u32 id)
{
	struct v4l2_control c;

	CLEAR(c);
	c.id = id;
	itd_xioctl(VIDIOC_G_CTRL, &c);
	print(1, "VIDIOC_G_CTRL[%s] = %i\n", get_control_name(id), c.value);
	return c.value;
}

static void itd_v4l2_s_ext_ctrl(__u32 id, __s32 val)
{
	struct v4l2_ext_controls cs;
	struct v4l2_ext_control c;

	CLEAR(cs);
	cs.ctrl_class = V4L2_CTRL_ID2CLASS(id);
	cs.count = 1;
	cs.controls = &c;

	CLEAR(c);
	c.id = id;
	c.value = val;

	print(1, "VIDIOC_S_EXT_CTRLS[%s] = %i\n", get_control_name(id),
	      c.value);
	print(2, "< ctrl_class: 0x%08X\n", cs.ctrl_class);
	print(2, "< count:      %i\n", cs.count);
	print(2, "< controls:   %p\n", cs.controls);
	print(2, "<< id:        0x%08X\n", c.id);
	print(2, "<< value:     %i\n", c.value);
	itd_xioctl(VIDIOC_S_EXT_CTRLS, &cs);
}

static int itd_v4l2_query_ctrl(__u32 id, int errout)
{
	struct v4l2_queryctrl q;
	int r = 0;

	CLEAR(q);
	q.id = id;
	if (errout)
		itd_xioctl(VIDIOC_QUERYCTRL, &q);
	else
		r = itd_xioctl_try(VIDIOC_QUERYCTRL, &q);
	if (r)
		return r;

	print(1, "VIDIOC_QUERYCTRL[%s] =\n", get_control_name(id));
	print(2, "> type:    %i\n", q.type);
	print(2, "> name:    `%.32s'\n", q.name);
	print(2, "> limits:  %i..%i / %i\n", q.minimum, q.maximum, q.step);
	print(2, "> default: %i\n", q.default_value);
	print(2, "> flags:   %i\n", q.flags);
	return 0;
}

static __s32 itd_v4l2_g_ext_ctrl(__u32 id)
{
	struct v4l2_ext_controls cs;
	struct v4l2_ext_control c;

	CLEAR(cs);
	cs.ctrl_class = V4L2_CTRL_ID2CLASS(id);
	cs.count = 1;
	cs.controls = &c;

	CLEAR(c);
	c.id = id;

	itd_xioctl(VIDIOC_G_EXT_CTRLS, &cs);
	print(1, "VIDIOC_G_EXT_CTRLS[%s] = %i\n", get_control_name(id),
	      c.value);
	return c.value;
}

static int isident(int c)
{
	return isalnum(c) || c == '_';
}

static void itd_request_controls(const char *start)
{
	char *end, *value;
	bool ext, next;
	char op;
	__u32 id;
	int val;

	do {
		for (end = (char *)start; isident(*end); end++) ;
		value = end;
		ext = FALSE;
		if (*value == '+') {
			value++;
			ext = TRUE;
		}
		op = *value++;
		*end = 0;
		next = FALSE;
		id = get_control_id(start);
		if (op == '=') {
			/* Set value */
			for (end = value; isident(*end); end++) ;
			if (*end == ',')
				next = TRUE;
			if (*end)
				*end++ = 0;
			if (sscanf(value, "%i", &val) != 1)
				error("bad control value");
			if (ext)
				itd_v4l2_s_ext_ctrl(id, val);
			else
				itd_v4l2_s_ctrl(id, val);
		} else if (op == '?') {
			/* Get value */
			if (*value == ',')
				next = TRUE;
			end = value + 1;
			if (ext)
				itd_v4l2_g_ext_ctrl(id);
			else
				itd_v4l2_g_ctrl(id);
		} else if (op == '#') {
			/* Query control */
			if (*value == ',')
				next = TRUE;
			end = value + 1;
			itd_v4l2_query_ctrl(id, 1);
		} else
			error("bad request for control");
		start = end;
	} while (next);
}

static void itd_enumerate_controls(const char *unused)
{
	int id;

	for (id = V4L2_CID_BASE; id < V4L2_CID_LASTP1; id++) {
		itd_v4l2_query_ctrl(id, 0);
	}
	for (id = V4L2_CID_PRIVATE_BASE;; id++) {
		int r = itd_v4l2_query_ctrl(id, 0);
		if (r == -EINVAL)
			break;
	}
}

static void itd_subdev_frame_interval(const char *s)
{
	static const struct token_list list[] = {
		{'p', TOKEN_F_ARG, "pad", NULL},
		{'i', TOKEN_F_ARG | TOKEN_F_ARG2, "interval", NULL},
		TOKEN_END
	};
	struct v4l2_subdev_frame_interval p;

	CLEAR(p);

	while (*s && *s != '?') {
		int val[4];
		switch (token_get(list, &s, val)) {
		case 'p':
			p.pad = val[0];
			break;
		case 'i':
			p.interval.numerator = val[0];
			p.interval.denominator = val[1];
			break;
		}
	}

	if (*s == '?') {
		print(1, "VIDIOC_SUBDEV_G_FRAME_INTERVAL\n");
		itd_xioctl(VIDIOC_SUBDEV_G_FRAME_INTERVAL, &p);
	} else {
		print(1, "VIDIOC_SUBDEV_S_FRAME_INTERVAL\n");
	}

	print(2, ": pad:       %i\n", p.pad);
	print(2, ": interval:  %i/%i\n", p.interval.numerator,
	      p.interval.denominator);

	if (*s != '?') {
		itd_xioctl(VIDIOC_SUBDEV_S_FRAME_INTERVAL, &p);
	}
}

static void itd_output_name(const char *arg)
{
	vars.pipes[vars.pipe].output = strdup(arg);
	if (!vars.pipes[vars.pipe].output)
		error("out of memory");
	vars.save_images = TRUE;
}

static void delay(double t)
{
	struct timespec ts;
	int r;

	if (t > 0.0) {
		ts.tv_sec = (long)t;
		ts.tv_nsec = (long)((t - ts.tv_sec) * 1000000000.0 + 0.5);
	} else {
		/* Zero means some small delay, here 1/100 second */
		ts.tv_sec = 0;
		ts.tv_nsec = 10 * 1000 * 1000;
	}
	print(1, "SLEEP %li.%08li s\n", (long)ts.tv_sec, ts.tv_nsec);
	r = nanosleep(&ts, NULL);
	if (r != 0)
		error("nanosleep failed");
}

static void capture_buffer_write(struct capture_buffer *cb, char *name, int i)
{
	FILE *f;
	int r;
	static const char number_mark = '@';
	char b[256];
	char n[5];
	char *c;

	if (!name || !cb->image)
		return;

	if (strlen(name) + sizeof(n) >= sizeof(b))
		error("too long filename");

	sprintf(b, "%s_%dx%d_%03i.pgm",
		name, cb->pix_format.width, cb->pix_format.height, i);

	f = fopen(b, "wb");
	if (!f)
		error("can not open file `%s'", name);

	fprintf(f,
		"P5\n%d %d\n%d\n",
		cb->pix_format.width, cb->pix_format.height, 65535);

	r = fwrite(cb->image, cb->length, 1, f);
	if (r != 1)
		error("failed to write data to file");

	r = fclose(f);
	if (r != 0)
		error("failed to close file");

	print(1, "Writing buffer #%03i (%i bytes) format %s to `%s'\n", i,
	      cb->length, symbol_str(cb->pix_format.pixelformat, pixelformats),
	      b);

}

int v4l2_demo(void)
{
	int i;
	int ret = setjmp(vars.exception);
	if (ret)
		return ret;
	print(0, "v4l2_demo\n");

	ion.fd = ion_open();
	if ( ion.fd < 0 ) {
	    print(0, "Open fd for  ion memory management ERROR!!\n");
	    return -1;
	} else {
	    print(0, "Open fd for  ion memory management (fd=%d).\n", ion.fd);
	}

	_PAGE_SIZE = getpagesize();
	_PAGE_MASK = ~(_PAGE_SIZE - 1);

	memset(&vars, 0, sizeof(vars));
	if (gettimeofday(&vars.start_time, NULL) < 0)
		error("getting start time failed");
	vars.verbosity = 2;

	for (i = 0; i < MAX_PIPES; i++) {
		vars.pipes[i].fd = -1;
		vars.pipes[i].reqbufs.count = 2;
		vars.pipes[i].reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vars.pipes[i].reqbufs.memory = V4L2_MEMORY_MMAP;
	}
	vars.pipes[0].active = TRUE;

	itd_open_device("/dev/video4");

	itd_vidioc_querycap(NULL);

	itd_vidioc_enuminput(NULL);

	itd_vidioc_sg_input("0");

	itd_vidioc_sg_fmt("type=1,width=3264,height=2448,pixelformat=SGRBG10,sizeimage=15980544");
	//itd_vidioc_sg_fmt("type=1,width=3264,height=2448,pixelformat=NV12,sizeimage=11985408");
	itd_vidioc_reqbufs("type=1,count=2,memory=USERPTR");

	itd_output_name("/home/jeffy/testpic");

	itr_capture(1);

	itd_close_device(NULL);

	return 0;
}

int v4l2_cleanup(void)
{
	int i;
	int ret = setjmp(vars.exception);
	if (ret)
		return ret;

	print(0, "v4l2_cleanup\n");
	for (vars.pipe = 0; vars.pipe < MAX_PIPES; vars.pipe++) {
		/* Save images */
		for (i = 0; i < vars.pipes[vars.pipe].num_capture_buffers; i++)
			capture_buffer_write(&vars.pipes[vars.pipe].
					     capture_buffers[i],
					     vars.pipes[vars.pipe].output, i);
	}

	for (vars.pipe = 0; vars.pipe < MAX_PIPES; vars.pipe++) {
		/* Stop streaming */
		if (vars.pipes[vars.pipe].streaming)
			itd_streamon((char *)FALSE);

		/* Free memory */
		itd_vidioc_querybuf_cleanup();
		for (i = 0; i < vars.pipes[vars.pipe].num_capture_buffers; i++) {
			free(vars.pipes[vars.pipe].capture_buffers[i].image);
		}
		itd_close_device(NULL);
		free(vars.pipes[vars.pipe].output);
	}

	close(ion.fd);

	return 0;
}

/* Beginning of ION memory manager.*/
int ion_open()
{
	int fd = open("/dev/ion", O_RDWR);
	if (fd < 0)
		error("open /dev/ion failed!\n");
	return fd;
}

static int ion_ioctl(int fd, int req, void *arg)
{
	int ret = ioctl(fd, req, arg);
	if (ret < 0) {
		error("ioctl %x failed with code %d: %s\n", req,
		      ret, strerror(errno));
		return -errno;
	}
	return ret;
}

int ion_alloc(int fd, size_t len, size_t align, unsigned int heap_mask,
	      unsigned int flags, ion_user_handle_t * handle)
{
	int ret;
	struct ion_allocation_data data = {
		.len = len,
		.align = align,
		.heap_id_mask = heap_mask,
		.flags = flags,
	};
	print(0, "run ion_alloc\n");
	ret = ion_ioctl(fd, ION_IOC_ALLOC, &data);
	if (ret < 0) {
		error("ion_ioct ION_IOC_ALLOC failed.\n");
		return ret;
	}
	*handle = data.handle;
	return ret;
}

int ion_free(int fd, ion_user_handle_t handle)
{
	struct ion_handle_data data = {
		.handle = handle,
	};
	return ion_ioctl(fd, ION_IOC_FREE, &data);
}

int ion_map(int fd, ion_user_handle_t handle, size_t length, int prot,
	    int flags, off_t offset, unsigned char **ptr, int *map_fd)
{
	struct ion_fd_data data = {
		.handle = handle,
	};

	int ret = ion_ioctl(fd, ION_IOC_MAP, &data);
	if (ret < 0) {
		error("ion_ioctl ION_IOC_MAP failed.\n");
		return ret;
	}
	*map_fd = data.fd;
	if (*map_fd < 0) {
		error("map ioctl returned negative fd\n");
		return -EINVAL;
	}
	*ptr = mmap(NULL, length, prot, flags, *map_fd, offset);
	if (*ptr == MAP_FAILED) {
		error("mmap failed: %s\n", strerror(errno));
		return -errno;
	}
	return ret;
}

/* End of ION memory manager.*/

__attribute__ ((weak))
int main(int argc, char *argv[])
{
	int ret;

	print(1, "Starting %s\n", name);
	name = argv[0];
	if (v4l2_demo() < 0)
		return 1;

	if (v4l2_cleanup() < 0)
		return 1;

	return ret;
}

/* EOF */
