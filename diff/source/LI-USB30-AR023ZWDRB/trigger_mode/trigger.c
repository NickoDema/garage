#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
//#include <linux/libv4l2.h>

#include <linux/uvcvideo.h>

#define CY_FX_UVC_XU_TRIGGER_ENABLE (0x0b00)
#define CY_FX_UVC_XU_SET_DELAY (0x0a00)

/**
 * Open the V4L2 device node with the given name.
 */

static  __u8 value[64] = {0};
static  __u8 * data = (__u8 *)value;
static  __u32 v4l2_dev;

struct uvc_xu_control_query xu_query = {
		.unit		= 3, //has to be unit 3
		.selector	= 1, //TD
		.query		= UVC_SET_CUR,
		.size		= 4, //TD
		.data		= value,
	};

int open_v4l2_device(char *device_name)
{
	int v4l2_dev;
	char *dev_node;

	if(device_name == NULL)
		//return C_INVALID_ARG;
		return -5;

	dev_node = (char *)malloc(5 + strlen(device_name) + 1);
	if(!dev_node)
		return 0;
	sprintf(dev_node, "/dev/%s", device_name);
	v4l2_dev = open(dev_node, 0);
	free(dev_node);
	return v4l2_dev;
}

void error_handle()
{
		int res = errno;

		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:		err = strerror(res); break;
		}
		printf("failed %s. (System code: %d) \n\r",
				err, res);

		return ;
	}

void TriggerMode_switch()
{

    sleep(1);
    //
    xu_query.selector = CY_FX_UVC_XU_SET_DELAY>>8;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 4;
    //
    // //Trigger mode enable
    if(ioctl(v4l2_dev, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();
    printf("Delay set to 0\n\r");

    sleep(1);
    //
    xu_query.selector = CY_FX_UVC_XU_TRIGGER_ENABLE>>8;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 2;
    //
    // //Trigger mode enable
    value[0] = 0x01;
    value[1] = 0x00;
    if(ioctl(v4l2_dev, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();
    printf("Trigger Mode ON\n\r");

	// sleep(4);
    //
    //Trigger mode disable, auto streaming
    // value[0] = 0x00;
    // value[1] = 0x00;
    // if(ioctl(v4l2_dev, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();
    // printf("Trigger Mode OFF\n\r");

}


void main()
{
    v4l2_dev = open_v4l2_device("video0");
    if(v4l2_dev < 0)
    {
        printf("open camera failed,err code:%d\n\r",v4l2_dev);
        return ;
    }

    TriggerMode_switch();

    close(v4l2_dev);

    return;

}
