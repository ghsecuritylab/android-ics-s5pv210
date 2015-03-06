//
//
//foat for g5 main.c
//
//

#include "foat.h"

#define GSM_MUXD "/system/bin/gsmmux"

int port_baudrate = 230400;
int *p_baudrate = &port_baudrate;
byte *tx_p=NULL;
static FILE *send_fp=NULL;
bool upgrade_begin_flag = FALSE;
bool upgrade_complete_flag = FALSE;
bool b5_flag = FALSE;
int percent_num = 0;
int swleng = 0;
void GetUpdataPercent();
pthread_t b5_thread;
pthread_mutex_t b5_mtx = PTHREAD_MUTEX_INITIALIZER;

GHT_GOAT_OP gtud_serial_op;
GHT_GOAT_OP *gtud=&gtud_serial_op;

void ght_send_start_cmd(void) //DWORD size
{
    byte at_cmd[]={0xb5};
    //printf("write 0xb5=====%x===%d\n",at_cmd[0],sizeof(at_cmd));
    ght_send_data(gtud->tty_fd,at_cmd,sizeof(at_cmd));
    //usleep(100*1000);
}

void ght_send_switch_cmd(void)
{
	byte at_cmd[2]={0xb5,0x08};
	if(*p_baudrate == B115200){
		printf("sync to 115200");
		at_cmd[0]=0xb5;
		at_cmd[1]=0x08;
	}
	else if(*p_baudrate == B460800){
		printf("sync to 460800");
		at_cmd[0]=0xb5;
		at_cmd[1]=0x0a;
	}
	else if(*p_baudrate == B230400){
		printf("sync to 460800");
		at_cmd[0]=0xb5;
		at_cmd[1]=0x09;
	}
	
	if(gtud->file_flag==TRUE)
	{
		at_cmd[0]=0xcc;
	}
	//printf("write baud sync=====%x===%d\n",at_cmd[0],sizeof(at_cmd));
   	ght_send_data(gtud->tty_fd,at_cmd,sizeof(at_cmd));
	usleep(100*1000);

}

void ght_send_dl_end_cmd()
{
   byte at_cmd[]={0xaa,0x05,0x00,0x00,0x00,0x05};
   ght_send_data(gtud->tty_fd,at_cmd,sizeof(at_cmd));
}

void ght_send_dl_end_rsp_cmd()
{
   byte at_cmd[]={0xaa,0x06,00,0x02,0x00,0x00,0x00,0x08};
   ght_send_data(gtud->tty_fd,at_cmd,sizeof(at_cmd));
}

void ght_send_dl_begin_cmd(unsigned long size_len)
{

    byte len=0;
    unsigned short sum=0;
    byte at_cmd[20]={0};
    byte at_cmd_start[]={0xaa,0x01,00,0x08,0x33,0x22,0x04,0x01};
    //print_system_time();
    memcpy(at_cmd,at_cmd_start,sizeof(at_cmd_start));
    len+=sizeof(at_cmd_start);
    at_cmd[len++]=(byte)((size_len>>24)&0xFF);
    at_cmd[len++]=(byte)((size_len>>16)&0xFF);
    at_cmd[len++]=(byte)((size_len>>8)&0xFF);
    at_cmd[len++]=(byte)((size_len)&0xFF);
    sum=ght_chksum(&at_cmd[1],(len-1));
    at_cmd[len++]=(sum>>8) & 0x00FF;
    at_cmd[len++]=sum& 0x00FF;
    ght_send_data(gtud->tty_fd,at_cmd,len);
    usleep(200*1000);
}

unsigned long ght_load_file(byte *file_buf)
{
	unsigned long file_len=0;
	int i;
	if(send_fp==NULL)
   {
       //AfxMessageBox("no files");
       printf("no files!Fibocom_upgrade");
       exit(1);
   }
   file_len=fread(file_buf,sizeof(byte),GHT_BLOCK_SIZE,send_fp); // FOAT_SEND_DATA_SIZE FOAT_BLOCK_SIZE
	//printf("read from file ======= %ld \n", file_len);
    //for(i=0;i<file_len;i++){
	//	printf("%x ",*(file_buf+i));
   // }
	//printf("\n");
   if(file_len==0)
   {


       //gtud->updata_status=foat_init;
	   if(tx_p!=NULL)
	   {
          free(tx_p);
	   }
       tx_p=NULL;
		if(send_fp != NULL){
	   		fclose(send_fp);
			send_fp=NULL;
			}
       return 0;
   }
	return file_len;
}

unsigned long ght_send_dl_data_cmd(byte *send_p,unsigned long sn)
{
	int i;
    unsigned short sum=0;
    unsigned long load_size=0,len=0;
    byte data_head[]={0xaa,0x03};
    memcpy(send_p,data_head,sizeof(data_head));
    len+=sizeof(data_head);
    len+=2;
    send_p[len++]=(byte)((sn>>24)&0xFF);
    send_p[len++]=(byte)((sn>>16)&0xFF);
	send_p[len++]=(byte)((sn>>8)&0xFF);
    send_p[len++]=(byte)((sn)&0xFF);
	load_size=ght_load_file(&send_p[len]);
	if(load_size==0)
	{
	    return 0;
	}

    //send_p[sizeof(data_head)]=(byte)(((load_size+4)>>24)&0xFF);

    //send_p[sizeof(data_head)+1]=(byte)(((load_size+4)>>16)&0xFF);
    send_p[sizeof(data_head)]=(byte)(((load_size+4)>>8)&0xFF);
	send_p[sizeof(data_head)+1]=(byte)((load_size+4)&0xFF);
    len+=load_size;
	sum=ght_chksum(&send_p[1],(len-1));
    send_p[len++]=(sum>>8) & 0x00FF;
    send_p[len++]=sum& 0x00FF;
    ght_send_data(gtud->tty_fd,send_p,len);
    //for(i=0;i<len;i++){
	//	printf("%x ",*(send_p+i));
    //}
	//printf("\n");
    return len;
}

void ght_foat_rx_process(byte *rx_data)
{
	if((foat_request==gtud->updata_status)&&(*rx_data==0x5b)&&(*(rx_data+1)!=0))
	{
		//close_tx_Thread();
		//Sleep(100);
		gtud->updata_status=foat_updating;
		gtud->packet_num=0;
		printf("Upgrade Begin!......\n");
		upgrade_begin_flag = TRUE;
		ght_send_dl_begin_cmd(gtud->file_size);
		if(tx_p==NULL)
		{
			tx_p=(byte *)malloc(GHT_SEND_SIZE);
		}
		memset(tx_p,0,sizeof(GHT_SEND_SIZE));
		send_fp=fopen("foat.bin", "rb");
		if (send_fp == NULL)
		{
			free(tx_p);
			tx_p = NULL;
			//AfxMessageBox("no files");
			printf("No files!!");
			exit(1);
		}
		printf("Going to download data...\n");
	}
	else if(*rx_data==0x5b)
	{
		//CFoatDlg dlg;
		b5_flag = FALSE;
		usleep(100*1000);
		ght_send_switch_cmd();
	}
	else if(foat_updating==gtud->updata_status) //()&&(0xaa==*rx_data)
	{
		const byte at_cmd_dl_begin_rsp[]={0xaa,0x02,00,01,00};
		const byte at_cmd_dl_data_rsp[]={0xaa,0x04,00,05,00};
		const byte at_cmd_dl_end[]={0xaa,0x05};
		const byte at_cmd_dl_end_rsp[]={0xaa,0x06};
		
		if((memcmp(rx_data,at_cmd_dl_begin_rsp,sizeof(at_cmd_dl_begin_rsp))==0)||
		   (memcmp(rx_data,at_cmd_dl_data_rsp,sizeof(at_cmd_dl_data_rsp))==0))
		{	
			 if(ght_send_dl_data_cmd(tx_p,gtud->packet_num++)==0)
			 {
			 //if no more data to write ,close the device port & notify complete
				 if(tx_p!=NULL)
				 {
					free(tx_p);
					tx_p=NULL;
				 }
				 gtud->updata_status=foat_complete;
				 ght_send_dl_end_cmd();
				 if(send_fp !=NULL){
				 	printf("send_fp !=NULL\n");
				 	fclose(send_fp);

					send_fp=NULL;
				 	}
				 printf("Upgrade Complete!\n");
				 upgrade_begin_flag = FALSE;
				 upgrade_complete_flag = TRUE;
				 if (gtud->tty_fd)		   
				 {
					 close(gtud->tty_fd);
					// gtud->tty_fd = NULL;
					 usleep(500*1000);
				 } 
				 return;
			 }
			 memset(tx_p,0,sizeof(GHT_SEND_SIZE)); //clean the buff of tx_p
		}
		else if(memcmp(rx_data,at_cmd_dl_end,sizeof(at_cmd_dl_end))==0)
		{
			if(send_fp !=NULL){
			   fclose(send_fp);
			   send_fp=NULL;

			   }
			gtud->updata_status=foat_complete;
			ght_send_dl_end_rsp_cmd();
			//AfxMessageBox("Fail,stop!");
			printf("Fail,stop!!\n");
			if(tx_p!=NULL)
			{
			   free(tx_p);
			   tx_p=NULL;
			}
			exit(1);
		}
		else if((memcmp(rx_data,at_cmd_dl_end_rsp,sizeof(at_cmd_dl_end_rsp))==0)||
				(rx_data == NULL))
		{
			return;
		}
		else
		{
			if(send_fp !=NULL){
			   fclose(send_fp);
			   send_fp=NULL;
			   }
			gtud->updata_status=foat_complete;
			ght_send_dl_end_cmd();
			//AfxMessageBox("Wrong data\A3\ACstop!");
			printf("Wrong data stop!\n");
            exit(1);
		}
	}
}

static void * ght_foat_rev_str(void *param)
{
    DWORD   BytesRead,mev = 0;
    byte    RXBuff[513]={0}; /* 520 */
    BOOL    bResult = TRUE;
	byte b5_cmd[]={0xb5};
    int ret = 0;
    int i=0;
    fd_set rfds;
    struct timeval tv={0,0};

	b5_flag = TRUE;
    while (!upgrade_complete_flag) 
    {
        if (gtud->tty_fd <0) 
        {
			upgrade_complete_flag = TRUE;
            usleep(1000*1000);
            continue;
        }
		if(b5_flag){			
			write(gtud->tty_fd, b5_cmd, sizeof(b5_cmd)); //send b5
			usleep(200*1000);
		}
    	FD_ZERO(&rfds);
    	FD_SET(gtud->tty_fd, &rfds);
    	if (select(1+gtud->tty_fd, &rfds, NULL, NULL, &tv)>0)
		{
			if (FD_ISSET(gtud->tty_fd, &rfds))
			{
		        BytesRead = 0;
				usleep(100*1000);
		        BytesRead = read(gtud->tty_fd, RXBuff,sizeof(RXBuff)); //256
		        if (BytesRead) 
		        {
		            ///*	
					//for(i=0;i<30;i++){
						//printf("%x ",RXBuff[i]);

					//}
					//printf("\n",RXBuff[i]);
					//*/
		        }
		        if(RXBuff!=NULL)
		        {	
					if(upgrade_begin_flag){
						GetUpdataPercent();  //print the upgrade process percent
					}
					//printf("BytesRead ============= %d\n",BytesRead);
					RXBuff[BytesRead] = '\0';
		            ght_foat_rx_process(RXBuff);
		            memset(RXBuff,0,sizeof(RXBuff));
		        }
        }
    }
}
	return NULL;
}


int foat_create_Thread()

{   // DWORD threadID;
    //DWORD code;
   // int err;
   // err = pthread_create(&gtud->thread, NULL, ght_foat_rev_str, NULL);
    int ret;
    pthread_t tid;
    pthread_attr_t attr;

    pthread_attr_init (&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&gtud->thread, &attr, ght_foat_rev_str, &attr);

    if (ret < 0) {
        perror ("pthread_create");
        return -1;
    }
    return 0;
}

void ght_lod_file_process()
{
    FILE *fp_lod=NULL;
    FILE *fp_bin=NULL;
    byte  start_write_file=0;
    char  lod_buf[30]={0};
    byte  *bin_buf=NULL;
    char *lod_path=NULL;
    const byte lod_flag[]={0x40,0x30,0x38,0x30,0x31,0x30,0x30,0x30,0x30}; //@08010000
    const byte end_flag[]={0x23,0x63,0x68,0x65,0x63,0x6B,0x73,0x75,0x6D,0x3D}; //#checksum=1ba21100
    //printf("upgrade_fils \"/sdcard/G520app.lod\" \n");
    lod_path=gtud->file_applod;
    printf("upgrade_fils \"%s\" \n", lod_path);
    //fp_lod= fopen("/sdcard/G520app.lod", "rb");
    fp_lod= fopen(lod_path, "rb");
    if (fp_lod == NULL)
    {
       //AfxMessageBox("No lod file");
		printf("No lod file!\n");
       exit(1);
    }
    bin_buf=(byte *)malloc(0x800000);
    if(bin_buf==NULL)
    {
        fclose(fp_lod);
        return;
    }
	system("rm foat.bin");
	sleep(1);
    fp_bin = fopen("foat.bin","wb");
    if (fp_bin == NULL)
    {
        fclose(fp_lod);
        free(bin_buf);
        bin_buf=NULL;
		printf("Binary file open error!\n");
        exit(1);
    }
	gtud->file_size=0;
    while (!feof(fp_lod)) 
    {
        //fgets((char *)lod_buf,sizeof(lod_buf)-1, fp_lod);
        if(NULL == fgets((char *)lod_buf,sizeof(lod_buf)-1, fp_lod))
        {
            break;        
        }
	    if(memcmp(lod_buf,end_flag,sizeof(end_flag))==0)
		{
			start_write_file=0;
			if(gtud->file_size)
			{
				byte chksum[4]={0};
				ght_string_asc2hex(chksum,&lod_buf[10],8);
				//gtud->file_chksum=*((unsigned int *)(chksum));
                gtud->file_chksum=((chksum[0]<<24)&0xFF000000)|

					              ((chksum[1]<<16)&0x00FF0000)|

								  ((chksum[2]<<8)&0x0000FF00)|

                                  ((chksum[3])&0x000000FF);

                fwrite(bin_buf,sizeof(char), gtud->file_size,fp_bin);
			}
			break;
		}
        else if(start_write_file)
		{
			byte block_str1[]={0x40,0x30,0x38};
			int line_size=0;
            if(memcmp(lod_buf,block_str1,sizeof(block_str1))==0)
			{
				continue;
			}
            line_size=strlen(lod_buf);
            line_size=ght_string_asc2hex(&bin_buf[gtud->file_size],lod_buf,(line_size-1));
			//memcpy(&bin_buf[gtud->file_size],lod_buf,8);
            gtud->file_size+=line_size;
		}
        else if(memcmp(lod_buf,lod_flag,sizeof(lod_flag))==0)
        {
            start_write_file=1;
		}
        memset(lod_buf,0,sizeof(lod_buf));
    }
    fclose(fp_lod);
    fclose(fp_bin);
	if(bin_buf!=NULL)
	{
       free(bin_buf);
	}
    bin_buf=NULL;
	//printf("fileSize ==== %ld\n", gtud->file_size);
	swleng = (gtud->file_size)/(GHT_BLOCK_SIZE);
	if(gtud->file_size==0)
	{
		//deleteFile(file_name);

        //AfxMessageBox("File wrong");
		printf("File wrong\n");
		exit(1);
	}
}

void GetUpdataPercent() 
{
	//int swleng = 0;
	int percent = 0;

	if((0 != gtud->file_size)&&(gtud->file_size < (GHT_BLOCK_SIZE)))  //Customer APP
	{
		percent_num = 100; //percent_num + 20;
		//m_progress.SetPos(percent_num);
		sleep(1);
	}
	/*else if(upgrade_complete_flag)   //updata Customer app "460800"
	{
		percent_num = 100;
		m_progress.SetPos(percent_num);
		Sleep(1000);
		upgrade_complete_flag = FALSE;
	}*/
	else   //SW app.lod file
	{
		//swleng = (gtud->file_size)/(GHT_BLOCK_SIZE);
		percent = (int)((gtud->packet_num)*100);
		if((percent!=0) && (swleng !=0))
		{
			percent_num = percent/swleng;
		}
	}
	if(percent_num > 100){
		percent_num =100;
	}
	printf("\rUpgrade process: %d%%",percent_num);
	fflush(stdout);
}


int get_pid(char *name)
{
    char cmd[20] = { 0 };
    char szbuf[100] = { 0 };
    char *p_pid = NULL;
    FILE *pFile = NULL;
    int pid = 0;
 
    sprintf(cmd, "ps a");
    pFile = popen(cmd, "r");
    if (pFile != NULL) {
        while (fgets(szbuf, sizeof(szbuf), pFile)) {
            if (strstr(szbuf, name)) {
                p_pid = strstr(szbuf, " ");
                pid = strtoul(p_pid, NULL, 10);
                printf("--- gsmmux pid = %d ---\n", pid);
                break;
            }
        }
    }
 
    pclose(pFile);
    return pid;
}
 

void ril_kill_gsmmux()
{
    int pppd_pid ;
    int count = 5;
 
 while(count >= 0)
    {  
        pppd_pid = get_pid(GSM_MUXD);
        if (pppd_pid) 
        {
           kill(pppd_pid, SIGTERM);
           sleep(5);
           count--;
        }
        else
        {
            break;
        }
    }
    printf("gsmmux killed %d times\n", 5-count);
}

enum rfkill_type {
	RFKILL_TYPE_ALL = 0,
	RFKILL_TYPE_WLAN,
	RFKILL_TYPE_BLUETOOTH,
	RFKILL_TYPE_UWB,
	RFKILL_TYPE_WIMAX,
	RFKILL_TYPE_WWAN,
	RFKILL_TYPE_GPS,
	RFKILL_TYPE_FM,
	NUM_RFKILL_TYPES,
};

enum rfkill_operation {
	RFKILL_OP_ADD = 0,
	RFKILL_OP_DEL,
	RFKILL_OP_CHANGE,
	RFKILL_OP_CHANGE_ALL,
};

struct rfkill_event {
	__u32 idx;
	__u8  type;
	__u8  op;
	__u8  soft, hard;
} __packed;

static int power_modem(int on)
{
	int fd;
	struct rfkill_event rfk;
	int written, cur = 0, len = sizeof(rfk);
	unsigned char *data;
	
	fd = open("/dev/rfkill", O_RDWR);
	if (fd >= 0) {
		memset(&rfk, 0, sizeof(rfk));
		rfk.type = RFKILL_TYPE_WWAN;
		rfk.op = RFKILL_OP_CHANGE_ALL;
		rfk.soft = on ? 0 : 1;
		data = (unsigned char *)&rfk;
		while (cur < len) {
			do {
				written = write (fd, data + cur, len - cur);
			} while (written < 0 && errno == EINTR);
		
			if (written < 0) {
				break;
			}				
			cur += written;
		}
		
		close(fd);
		if (cur == len) return 0;

		return -1;
	}
	else{
		 printf("open /dev/rfkill fail !!!");
	}

	return -ENOENT;
}


int main(int argc, char **argv)
{
	int err;
	int opt;
	int n;
	char buf[80];
	//chdir("/sdcard");
	getcwd(buf, sizeof(buf));
	printf("current working directory : %s\n", buf);
	//*p_baudrate = 0;
	while ( -1 != (opt = getopt(argc, argv, "d:s:b:f:"))) {
        switch (opt) {  
		 	case 'd':
				strcpy(gtud->dev, optarg);
				printf("upgrade port is  %s \n", gtud->dev);
				break;
			case 's':
				*p_baudrate = atoi(optarg);
                break;
			case 'b':
				printf("case b optarg is %s \n", optarg);
				strcpy(gtud->file_bin, optarg); 	 
				printf("bin_fils %s \n", gtud->file_bin);
				gtud->file_size=0;
				//ght_foat_bin_file_process();
				break;
			case 'f':
				strcpy(gtud->file_applod, optarg); 	 
				//printf("upgrade_fils %s \n", gtud->file_applod);
				gtud->file_size=0;
				break;				
           	default:
				break;
        	    }

    }
	ril_kill_gsmmux();
	sleep(1);
	if(get_pid(GSM_MUXD)){
		printf("gsmmux couldn't end ,exit!!");
		//exit(1);
	}
	//open and config port
	gtud->tty_fd = open(gtud->dev, O_RDWR|O_NOCTTY);
		if(gtud->tty_fd < 0){
		printf("Open Port Failed!!Check it......by Trento\n");
		exit(1);
	}
	set_speed(gtud->tty_fd, p_baudrate);
    //printf("Open Port success!!\n*** Module is Powering OFF ***\n");
	//system("echo 0 > /sys/class/sw_2g_module/modem/modem_power");
	//sleep(1);	
	ght_lod_file_process();

	foat_create_Thread();
	sleep(1);
#if 1
	//b5_flag = TRUE;
	//foat_send_b5_Thread();
	//pthread_join(b5_thread,NULL);
#else
    printf("Reseting module......\n");
	err = writeline(gtud->tty_fd, "AT+CFUN=15\r\n");
	if (err < 0){
		printf("write error!!\n");
	}
	usleep(900*1000);	
	//printf("writing b5!\n");
	ght_send_start_cmd(); //send b5
#endif
	gtud->updata_status=foat_request;
	printf("Upgrade thread start...\n*** Powering ON the module... ***\n");
	power_modem(0);
	power_modem(1);	
	//system("echo 1 > /sys/class/sw_2g_module/modem/modem_power");
	while(!upgrade_complete_flag);

	//pthread_join(gtud->thread,NULL);
	//system("rm foat.bin");  
	if(gtud->tty_fd > 0){
		close(gtud->tty_fd);
		printf("Port closed.\n");
	}  
	printf("-- main process end --\n");

	exit(EXIT_SUCCESS);
	

}

