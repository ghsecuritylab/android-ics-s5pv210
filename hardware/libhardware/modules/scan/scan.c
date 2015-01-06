#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h> 
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>

#include <hardware/hardware.h>
#include <hardware/scan.h>

#define LOG_TAG "ScanHALStub"

#include <cutils/log.h>
#include <cutils/atomic.h>

#define SCAN_DEBUG	0
#if SCAN_DEBUG
        #define DEBUG   LOGE
#else
        #define DEBUG(...)
#endif

#define MODULE_NAME "Scan"
#define MODULE_AUTHOR "chenguanhua@vanstone.com.cn"

#define CHECK_ACK()		\
	if(response.prefix.opcode != CMD_ACK){\
		LOGE("error. response.prefix.opcode=%#x, should be %#x\n",\
			response.prefix.opcode, CMD_ACK);\
		return SSI_CMDERR;\
	}\

static unsigned short complement(unsigned short data)
{
	return ( (unsigned short)(~data) +1);
}

static unsigned short SSI_checksum(unsigned char* data, unsigned char len)
{
	unsigned char i;
	unsigned short value = 0;

	for (i = 0; i < len; i++){
		value += data[i];
	}

	value =complement(value);
	return value;
}

static int SSI_open(void)
{
	int fd;
	struct termios scan_termios;

 	fd = open("/dev/ttyXM0",O_RDWR);
// 	fd = open("/dev/s3c2410_serial0",O_RDWR);
	if(fd < 0){
		LOGE("open /dev/ttyXM0 failed\n");
		return SSI_BUSBUSY;
	}

	DEBUG("port setup start...\n");

	tcgetattr(fd,&scan_termios);
	cfmakeraw(&scan_termios);
	cfsetispeed(&scan_termios,B9600);
	cfsetospeed(&scan_termios,B9600);
	tcsetattr(fd,TCSANOW,&scan_termios);
	tcflush(fd, TCIOFLUSH);

	DEBUG("port setup finished...\n");

	return fd;
}

static int SSI_close(int fd)
{
	int ret = SUCCESS ;

	close(fd);
	DEBUG("close port\n");

	return ret;
}

static int read_poll(int fd, int rdwr, long timeout)
{
	fd_set fds;
	struct timeval tv;

	tv.tv_sec = timeout / 1000;
	tv.tv_usec = (timeout % 1000) * 1000;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	return select(fd + 1, rdwr ? &fds : NULL, rdwr ? NULL : &fds, NULL, &tv);
}

static int read_timeout(int fd, unsigned char *data, int len, long timeout)
{
	int ret = -1;

	if(timeout <= 0){
		return read(fd, data, len);
	}
	else{
		ret = read_poll(fd, 1, timeout);
		if(ret > 0){
			ret = read(fd, data, len);
		}
		else{
			return ret;
		}

		return ret;
	}
}

static int host_transmit(int fd, struct SSI_command_t *command)
{
	int ret, len;
	unsigned char i = 0, buf[MAX_TX_BUF_LEN];

	buf[i++] = command->prefix.length;
	buf[i++] = command->prefix.opcode;
	buf[i++] = command->prefix.message_source;
	buf[i++] = command->prefix.status;

	len = command->prefix.length - 4;
	if(len){
		if(len > MAX_TX_DATA_LEN){
			return SSI_TX_OVERFOLW;
		}
		else{
			memcpy(buf + i, command->data, len);
			i += len;
		}
	}
	
	command->suffix.checksum = SSI_checksum(buf, i);
	buf[i++] = (command->suffix.checksum >> 8) & 0xFF;
	buf[i++] = (command->suffix.checksum) & 0xFF;

	ret = write(fd, buf, i);
	if(ret != i){
		LOGE("write command fail.ret=%d, i=%d\n", ret, i);
		return SSI_COMMERR;
	}

	return SUCCESS;
}

static int host_receive(int fd, struct SSI_response_t *response)
{
	int ret, i, len;
	unsigned short checksum;
	long number;
	unsigned char buf[MAX_RX_BUF_LEN + 1];
	
	memset(buf, 0, sizeof(buf));

	ret = read_timeout(fd, buf, 1, READ_TIME_OUT);
	if(ret != 1){
		LOGE("host_receive data len fail ret=%d\n", ret);
		return SSI_COMMERR;
	}
	DEBUG("host_receive data len:%d\n", buf[0]);

	if(((buf[0] + SUFFIX_LEN) - 1) > MAX_RX_BUF_LEN){
		return SSI_RX_OVERFOLW;
	}
	else{
		ret = read_timeout(fd, buf + 1, ((buf[0] + SUFFIX_LEN) - 1), READ_TIME_OUT);
	}

#if SCAN_DEBUG
	for(i = 0; i <  buf[0]; i++)
		LOGE("buf[%d]=%#x, ", i, buf[i]);
	LOGE("\n");
#endif
	if(ret != ((buf[0] + SUFFIX_LEN) - 1)){
		LOGE("host_receive packet fail. ret=%d\n", ret);
		return SSI_COMMERR;
	}
 	response->prefix.length = buf[0];
	response->prefix.opcode = buf[1];
	response->prefix.message_source = buf[2];
	response->prefix.status = buf[3];
	len = response->prefix.length - 4;
	if(len){
		memcpy(response->data, buf + 4, len);
	}

	response->suffix.checksum = (
		(buf[(unsigned char)response->prefix.length] << 8) | 
		buf[(unsigned char)response->prefix.length + 1]);

	checksum = SSI_checksum(buf, response->prefix.length);
	if(response->suffix.checksum != checksum){
		LOGE("host_receive checksum fail\n");
		return SSI_CHECKSUMERR;
	}
	return SUCCESS;
}

static int host_transceive(struct SSI_command_t *command, struct SSI_response_t *response)
{
	int fd, ret ;

	if((fd = SSI_open()) < 0){
		return SSI_BUSBUSY; 
	}

	ret = host_transmit(fd, command);
	if(ret){
		SSI_close(fd);
		return ret;
	}

	ret = host_receive(fd, response);
	if(ret){
		SSI_close(fd);
		return ret ;
	}

	if(SSI_close(fd)){
		return SSI_BUSBUSY; 
	}

	return ret;
}

static int SSI_get_version(unsigned char *version)
{
	int ret; 
	struct SSI_command_t command;
	struct SSI_response_t response;

	command.prefix.length = 4;
	command.prefix.opcode = REQUEST_REVISION;
	command.prefix.message_source = HOST;
	command.prefix.status =0;

	ret = host_transceive(&command, &response);
	if(ret){
		return ret;
	}

	if(response.prefix.opcode != REPLY_REVISION){
		LOGE("error. response.prefix.opcode=%#x, should be %#x\n",
			response.prefix.opcode, REPLY_REVISION);
		return SSI_CMDERR;
	}
#if SCAN_DEBUG
	{
		int i;
		for(i=0; i<response.prefix.length - 4; i++){
			LOGE("response.data[%d]=%c\n", i, response.data[i]);
		}
	}
#endif
//	memcpy(version, response.data, response.prefix.length - 4);
//	version[(unsigned char)response.prefix.length - 4] = '\0';
	memcpy(version, response.data, 10);
	version[10] = '\0';

	DEBUG("version:%s\n", version);
	return ret;
}

static int SSI_scan_enable(int enable)
{
	int ret; 
 	struct SSI_command_t command;
	struct SSI_response_t response;

	command.prefix.length = 4;
	command.prefix.opcode = enable == 1 ? SCAN_ENABLE : SCAN_DISABLE; 
	command.prefix.message_source = HOST;
	command.prefix.status =0;

	ret = host_transceive(&command, &response);
	if(ret){
		return ret;
	}

	CHECK_ACK();

	return ret;
}

static int SSI_decode_enable(int enable)
{
	int ret; 
	struct SSI_command_t command;
	struct SSI_response_t response;
 
	command.prefix.length = 4;
	command.prefix.opcode = enable == 1 ? START_DECODE : STOP_DECODE; 
	command.prefix.message_source = HOST;
	command.prefix.status =0;

	ret = host_transceive(&command, &response);
	if(ret){
		return ret;
	}

	CHECK_ACK();

	return ret;
}

static int SSI_wakeup(void)
{
	int fd, ret ,i = 1;
	unsigned char wakeup[] = {0x00};

	if((fd = SSI_open()) < 0){
		return SSI_BUSBUSY; 
	}

	while(i--){
		ret = write(fd, wakeup, 1);
		if(ret != 1){
			LOGE("wakeup fail\n");
			break;
		}
		else if(!i){
			DEBUG("wakeup!\n");
		}
	}

	if(SSI_close(fd)){
		return SSI_BUSBUSY; 
	}
	return SUCCESS; 
}

static int SSI_param_send(unsigned char data[])
{
	int ret; 
	struct SSI_command_t command;
	struct SSI_response_t response;

	command.prefix.length = 7;
	command.prefix.opcode = PARAM_SEND; 
	command.prefix.message_source = HOST;
	command.prefix.status = 0;
	command.data[0]= 0xFF;//no beep
	command.data[1]= data[0];
	command.data[2]= data[1];

	ret = host_transceive(&command, &response);
	if(ret){
		return ret;
	}

	CHECK_ACK();

	return ret;
}

static int SSI_decode_data(struct barcode_t *barcode)
{
	int fd, ret;
	unsigned char buf[MAX_RX_BUF_LEN];	
	struct SSI_command_t command;

	memset(buf, 0, sizeof(buf));
	
	command.prefix.length = 4;
	command.prefix.opcode = CMD_ACK;
	command.prefix.message_source = HOST;
	command.prefix.status = 0;

	if((fd = SSI_open()) < 0){
		return SSI_BUSBUSY; 
	}

	barcode->len = read_timeout(fd, buf, MAX_RX_BUF_LEN, READ_TIME_OUT);
	if(barcode->len <= 0){
		LOGE("get decode data fail. barcode->len=%d\n", barcode->len);
		SSI_close(fd);
		return SSI_COMMERR;
	}

	ret = host_transmit(fd, &command);
	if(ret){
		SSI_close(fd);
		return ret;
	}

	if(SSI_close(fd)){
		return SSI_BUSBUSY; 
	}

//	barcode->type = (enum bar_type)buf[0];
//	memcpy(barcode->code, buf + 1, barcode->len);
	memcpy(barcode->code, buf, barcode->len);
	barcode->code[barcode->len] = '\0';

	DEBUG("decode data:%s\n", barcode->code);

	return SUCCESS;
}

static int SSI_decode_barcode(struct scan_device_t* dev, struct barcode_t *barcode)
{
	int ret, serial_sel_fd, save;
	unsigned char buf[MAX_RX_DATA_LEN];
	unsigned char param_send[2];

	serial_sel_fd = open("/dev/serial_sel", O_RDWR);
	if(serial_sel_fd < 0){
		LOGE("open /dev/serial_sel fail\n");
		return SSI_SEL_SERIAL_FAIL;
	}

	ioctl(serial_sel_fd, SERIAL_SEL_SCAN);
//	ioctl(serial_sel_fd, POWER_SCAN, 1);
//	usleep(1000);

	ret = SSI_wakeup();
	if(ret){
		LOGE("SSI_wakeup failed.ret:%d\n", ret);
		return ret;
	}

	usleep(40000);//must greater than 20ms

	ret = SSI_get_version(buf);
	if(ret){
		LOGE("SSI_get_version failed.ret:%d\n", ret);
		return ret;
	}
	printf("SSI_get_version:%s\n", buf);

	ret = SSI_scan_enable(1);
	if(ret){
		LOGE("SSI_scan_enable failed.ret:%d\n", ret);
		return ret;
	}

	param_send[0] = 0x8A;
	param_send[1] = 0x08;
	ret = SSI_param_send(param_send);
	if(ret){
		LOGE("SSI_param_send failed.ret:%d\n", ret);
		return ret;
	}

	ret = SSI_decode_enable(1);
	if(ret){
		LOGE("SSI_decode_enable failed.ret:%d\n", ret);
		return ret;
	}

	ret = SSI_decode_data(barcode);
	if(ret){
		LOGE("SSI_decode_data failed.ret:%d\n", ret);
		return ret;
	}

//	ioctl(serial_sel_fd, POWER_SCAN, 0);
	close(serial_sel_fd);
//	usleep(1000);

	return SUCCESS;
}

static int scan_device_close(struct hw_device_t* device) {
	struct scan_device_t* scan_device = (struct scan_device_t*)device;
	if(scan_device) {
		free(scan_device);
	}

	return 0;
}

static int scan_device_open(const struct hw_module_t* module, const char* id, struct hw_device_t** device) {
	if(!strcmp(id, SCAN_HARDWARE_DEVICE_ID)) {
		struct scan_device_t* dev;

		dev = (struct scan_device_t*)malloc(sizeof(struct scan_device_t));
		if(!dev) {
			LOGE("Failed to alloc space for scan_device_t.");
			return -EFAULT;	
		}

		memset(dev, 0, sizeof(struct scan_device_t));

		dev->common.tag = HARDWARE_DEVICE_TAG;
		dev->common.version = 0;
		dev->common.module = (hw_module_t*)module;
		dev->common.close = scan_device_close;
		dev->get_barcode = SSI_decode_barcode;
	
		*device = &(dev->common);

		LOGI("Open device file /dev/scan successfully.");	

		return 0;
	}

	return -EFAULT;
}

static struct hw_module_methods_t scan_module_methods = {
	open: scan_device_open
};

struct scan_module_t HAL_MODULE_INFO_SYM = {
	common: {
		tag: HARDWARE_MODULE_TAG,	
		version_major: 1,
		version_minor: 0,
		id: SCAN_HARDWARE_MODULE_ID,
		name: MODULE_NAME,
		author: MODULE_AUTHOR,
		methods: &scan_module_methods,
	}
};

