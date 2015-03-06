#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <termios.h>
//#include <sys/signal.h> //needed in linux

#define FOAT_DEBUG_FLAG  0

#define  FOAT_OPEN_COMM 0X01
#define  FOAT_LOAD_FILE 0X10
#define  FOAT_PREPARE 0X11
#define  GHT_BLOCK_SIZE 0x8000
#define  GHT_SEND_SIZE (GHT_BLOCK_SIZE+0x10)

#define MAX_LENGTH 256
#define MAX_RESPONSE 0x1000
#define P_LONG_TIME 2
#define P_SHORT_TIME 1

typedef enum e_Bool{false=0,FALSE=0,true=1,TRUE=1}BOOL;
typedef BOOL bool;
typedef unsigned char byte;
typedef unsigned int  DWORD;
typedef enum
{
	foat_init=0,
	foat_request,
	foat_updating,
	foat_send_cmd=4,
	foat_send_data,
	foat_send_delay,
	foat_complete,
	foat_fail

}GHT_FOAT_UPDATA_STATUS;


typedef struct{
	int tty_fd;
	char dev[128];
	char file_bin[128];
	char file_applod[128];
	FILE *bin_fp;
	bool file_flag;
	int baudrate;
	int foat_status;
	pthread_t     thread;
	GHT_FOAT_UPDATA_STATUS updata_status;
	unsigned long packet_len;
	unsigned long packet_chksum;
	unsigned long file_size;
	unsigned long file_chksum;
	unsigned long packet_num;
}GHT_GOAT_OP;

unsigned short ght_chksum(byte *data,DWORD len);

byte ght_hex2asc(byte hex_d);

DWORD ght_string_hex2asc(byte *src_hex,char *dst_asc,DWORD len);

byte ght_asc2hex(char asc);

//ffffffff->0xffffffff
DWORD ght_string_asc2hex(byte *hex,char *asc,DWORD len);

int ght_send_data(int dev_fd,byte *at_cmd, int len);

int writeline (int fd, const char *s);

void set_speed(int fd, int *p_baudrate);

