
#include "foat.h"

unsigned short ght_chksum(byte *data,DWORD len)
{
   unsigned short sum=0;
   DWORD i;
   byte *check_data=data;

   for (i=0; i<len; i++)
   {
       sum += *(check_data+i);
   }

   return sum;
}

byte ght_hex2asc(byte hex_d)
{
   byte asc_s=0;

   if(hex_d<0x0A)
   {
      asc_s=hex_d+0x30;
   }
   else if((hex_d>9)&&(hex_d<=0x0F))
   {
      asc_s=hex_d+0x37;
   }

   return asc_s;
}

DWORD ght_string_hex2asc(byte *src_hex,char *dst_asc,DWORD len)
{
   DWORD i;

   for(i=0;i<len;i++)
   {
      *dst_asc++=ght_hex2asc(((*src_hex)>>4)&0x0F);
      *dst_asc++=ght_hex2asc((*src_hex) & 0x0F);
      src_hex++;
   }
   //*dst_asc=NULL;

   return (2*i);
}


byte ght_asc2hex(char asc)
{
   byte hex=0;

   if((asc >='0')&&(asc <= '9'))
   {
      hex=asc-0x30;
   }
   else if((asc>='a')&&(asc <= 'f'))
   {
      hex=asc-0x57;
   }
   else if((asc>='A')&&(asc <= 'F'))
   {
      hex=asc-0x37;
   }

   return hex;
}

//ffffffff->0xffffffff
DWORD ght_string_asc2hex(byte *hex,char *asc,DWORD len)
{
   int i,j;
   byte *dst_hex=hex;
   char *src_asc=asc;

   for(i=(len-1),j=0;i>=0;j++)
   {
      *dst_hex++=(ght_asc2hex(src_asc[i-1])<<4)|ght_asc2hex(src_asc[i]); //3c0581c0
      //*dst_hex++=(ght_asc2hex(*src_asc)<<4)|ght_asc2hex(*(src_asc+1));
      //src_asc+=2;
	  i-=2;
   }

   return (j);
}
int ght_send_data(int dev_fd,byte *at_cmd, int len)
{   
    int written;
    int i;
    written = write(dev_fd, at_cmd, len);
    if(written < 256){
		printf("\nSend:\n[");
        for(i=0;i<written;i++){
			printf("%x ",*(at_cmd+i));
		}
		printf("]\n");
    }

    return written;

}

int writeline (int fd, const char *s)
{
    size_t cur = 0;
    size_t len = strlen(s);
    ssize_t written;
    if (fd < 0) {
        return -1;
    }
    printf("Send > %s\n", s);
    /* the main string */
    while (cur < len) {
        do {
            written = write (fd, s + cur, len - cur);
        } while (written < 0 && errno == EINTR);
        if (written < 0) {
            return -1;
        }
        cur += written;
    }
    return 0;
}

void set_speed(int fd, int *p_baudrate)

{
  	//int   i;
  	int   status;
	int baudrate;
	baudrate = *p_baudrate;
	printf("Baudrate Set to %d\n", baudrate);
	switch(baudrate) {
	    case 460800:
			baudrate = B460800;
			break;
		case 115200:
			baudrate = B115200;
			break;
		case 57600:
			baudrate = B57600;
			break;
		//case 56000:
		//	baudrate = B56000;
		//	break;
		case 38400:
			baudrate = B38400;
			break;
		case 19200:
			baudrate = B19200;
			break;
		//case 14400:
		//	baudrate = B14400;
		//	break;
		case 9600:
			baudrate = B9600;
			break;
		case 4800:
			baudrate = B4800;
			break;
		default: 	
			printf("Wrong Baudrate, Set to 115200\n");
			baudrate = B115200;
		}
	*p_baudrate = baudrate;
  	struct termios Opt;
  	tcgetattr(fd, &Opt);
   	tcflush(fd, TCIOFLUSH);
    cfsetispeed(&Opt, baudrate);

    cfsetospeed(&Opt, baudrate);

    Opt.c_lflag = 0;  //disable the flags,important
    Opt.c_oflag = 0; 
    Opt.c_cflag &= ~CRTSCTS;  //no flow control 
	Opt.c_cflag &= ~CSTOPB;  // 1 stop bit
	Opt.c_cflag &= ~(PARENB | PARODD);  //no par check disable
	Opt.c_cflag = (Opt.c_cflag & ~CSIZE) | CS8;
    Opt.c_iflag = IGNCR|IGNBRK;  
	Opt.c_iflag &= ~(IXON|IXOFF|IXANY);

    status = tcsetattr(fd, TCSANOW, &Opt);
	//printf("c_lflag == %\nc_oflag == %u\nc_iflag == %u\nc_cflag == %u\n",Opt.c_lflag,Opt.c_oflag,Opt.c_iflag,Opt.c_cflag);
    if  (status != 0){
    	perror("tcsetattr fd1");
   	    return;
     	}
   	tcflush(fd,TCIOFLUSH);
}



