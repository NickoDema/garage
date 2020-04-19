#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
// #include <sys/stat.h>
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

/**
 * Open the V4L2 device node with the given name.
 *
 * @param device_name	A device name as accepted by c_open_device()
 *
 * @return
 * 		- 0 if the device could not be opened
 * 		- a device handle > 0 on success
 */

static  __u8 value[64] = {0};
static  __u8 * data = (__u8 *)value;
static  __u32 v4l2_dev_0;
static  __u32 v4l2_dev_1;

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

		return;
	}



void TriggerMode_switch()
{

    // sleep(1);

    xu_query.selector = CY_FX_UVC_XU_TRIGGER_ENABLE>>8;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 2;

    //Trigger mode enable
    value[0] = 0x01;
    value[1] = 0x00;
    if(ioctl(v4l2_dev_0, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();
    if(ioctl(v4l2_dev_1, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();

	// sleep(4);
    //
    // //Trigger mode disable,auto streaming
    // value[0] = 0x00;
    // value[1] = 0x00;
    // if(ioctl(v4l2_dev, UVCIOC_CTRL_QUERY, &xu_query) != 0) error_handle();

    printf("Trigger Test DONE\n\r");

}


void main()
{
    v4l2_dev_0 = open_v4l2_device("video0");
    if(v4l2_dev_0 < 0)
    {
        printf("open camera failed,err code:%d\n\r",v4l2_dev_0);
        return ;
    }

    v4l2_dev_1 = open_v4l2_device("video1");
    if(v4l2_dev_1 < 0)
    {
        printf("open camera failed,err code:%d\n\r",v4l2_dev_1);
        return ;
    }

    TriggerMode_switch();

    close(v4l2_dev_0);
    close(v4l2_dev_1);

    return;

}
