#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<termios.h>
#include<fcntl.h>
#include<errno.h>

#define TRUE 1
#define FALSE -1
#define BUFF_MAXSIZE 2048

#define FREQUENCY_00 0      //设定的频率为0
#define FREQUENCY_05 5      //设定的频率为5
#define FREQUENCY_10 10     //设定的频率为10
#define FREQUENCY_20 20     //设定的频率为20
#define RESOLUTION_ONE_MM 1 //1表示选择设定的分辨率为1mm
#define RESOLUTION_Z_P_ONE_MM 2 //2表示选择设定的分辨率为0.1mm
#define MEASURING_POWER_ON 1    //1表示上电即测开启 
#define MEASURING_POWER_OFF 0   //0表示上电即测关闭

typedef unsigned char un_char;

//初始化设置，即设置通信协议
int OpenDev(char *dev);//打开串口设备文件
int set_speed(int fd, int speed, struct termios* newtio);//设置波特率
int Set_Parity(int fd, int databits, int stopbits, int parity);//设置数据位、停止位、校验位

//数据读写函数
int Write_Data(int fd, void *buf, int len);//发送命令代码函数
int Read_Data(int fd, char *buff);//接收命令代码函数

//模块的功能函数
int Open_LaserModule(int fd);//模块的打开
int Close_LaserModule(int fd);//模块的关闭
int Set_Address(int fd);//设置地址
un_char* Read_Parameter(int fd, un_char* device_parameter);//读取参数
un_char* Read_Device_Num(int fd, un_char* device_num);//读取机器号
int Distance_Modification(int fd, int decrease_or_increase, int distance_int);//距离修改,参数decrease_or_increase表示修正可选为取负或者取正，参数distance表示要修正的距离
int Mea_Interval(int fd, int interval_time_int);//连续测量时设置数据返回时间间隔,参数interval_time_int表示要设定的时间间隔为interval_time_int秒
int Distance_StartStop(int fd, int position_int);//设置距离起止点,参数position_int的值(1顶端算起；0加上模块长度+上面的距离修正)
int Set_MeasuringRange(int fd, int range);//设定量程,range表示要设定的量程大小05, 10, 30, 50, 80
int Set_Frequency(int fd, int freg);//设定频率,freg表示要设定的频率大小,00,05,10,20
int Set_Resolution(int fd, int mode);//设定分辨率,当mode=1表示设定的分辨率为1mm,当mode=2表示设定的分辨率为0.1mm
int Measuring_Power(int fd, int on_off);//设定上电即测,on_off=1表示开启该功能，on_off=0表示关闭该功能
int Single_Measurement_Broadcast(int fd);//单次测量（广播命令，返回结果存入模块缓存）
un_char* Read_Cache(int fd, un_char* cache_data);//读取缓存
un_char* Single_Measurement(int fd, un_char* single_mea);//单次测量
un_char* Continuous_Measurement(int fd, un_char* continuous_mea);//连续测量

int speed_arr[]={B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[]={115200, 38400, 19200, 9600, 4800, 2400, 1200, 300, 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300};


int main(int argc, char *argv[])
{
    int fd;
    un_char Buff_data_device[BUFF_MAXSIZE] = {0};//我只是测试用，就在栈分配的空间，正式编写一般需要自己分配动态内存
    struct termios oldtio, newtio;

    //打开串口
    char *dev = "/dev/ttyGS0";
    fd = OpenDev(dev);

    tcgetattr(fd, &oldtio);
    if(fd > 0)
    {
        set_speed(fd, 9600, &newtio);//设置9600bps波特率
    }else
    {
        printf("Can't Open Serial Port!/n");
        exit(0);
    }

    if(Set_Parity(fd, 8, 1, 'S') == FALSE)//调用设置8位数据位，1位停止位及无校验位
    {
        printf("Set Parityu Error!/n");
        exit(1);
    }

    //测试函数的调用   
    Open_LaserModule(fd);
    Set_Resolution(fd, RESOLUTION_Z_P_ONE_MM);
    Measuring_Power(fd, MEASURING_POWER_ON);
    Set_Frequency(fd, FREQUENCY_05);
    Set_MeasuringRange(fd, RANGE_80);

    sleep(5);
    Read_Parameter(fd, Buff_data_device);
    Set_Address(fd);
    Distance_Modification(fd, DISTANCE_IN, 2);
    Mea_Interval(fd, 5);
    Distance_StartStop(fd, 0);

    sleep(5);
    Read_Cache(fd, Buff_data_device); 
    sleep(5);
    Single_Measurement(fd, Buff_data_device);
    sleep(5);
    Continuous_Measurement(fd, Buff_data_device);

    Close_LaserModule(fd);
    close(fd);
}

//打开文件设备
int OpenDev(char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        printf("Can't Open Serial Port!\n");
        return FALSE;
    }else
        return fd;
}

//设置波特率
int set_speed(int fd, int speed, struct termios* newtio)
{
    int i;
    int status;
    struct termios* Opt = newtio;
    tcgetattr(fd, Opt);
    for(i = 0; i < sizeof(speed_arr)/sizeof(int); i++)
    {
        if(speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(Opt, speed_arr[i]);
            cfsetospeed(Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, Opt);
            if(status != 0)
            {
                printf("tcsetattr failed!\n");
                return FALSE;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
    return TRUE;
}

//设置fd的数据位、停止位、奇偶检验位
int Set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if(tcgetattr(fd, &options) != 0)
    {
        printf("Setup Serial 1 error!\n");
        return FALSE;
    }
    //对options的起始地址开始的termios结构体内存置零
    bzero(&options,sizeof(options));
    options.c_cflag &= ~CSIZE;

    //选择数据位
    switch(databits)
    {
        case 7:options.c_cflag |= CS7;
               break;
        case 8:options.c_cflag |= CS8;
               break;
        default:printf("Unsupported data size!/n");
                return FALSE;
    }

    //选择奇偶校验
    switch(parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~(PARENB);
            options.c_cflag &= ~(INPCK);
            break;
        case 'o'://偶校验
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_cflag |= INPCK;
            break;
        case 'e'://偶校验
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~(PARODD);
            options.c_cflag |= INPCK;
            break;
        case 's'://无奇偶校验
        case 'S':
            options.c_cflag &= ~(PARENB);
            options.c_cflag &= ~(CSTOPB);
            break;
        default:
            printf("Unsupported parity!/n");
            return FALSE;
    }

    //选择停止位
    switch(stopbits)
    {
        case 1:
            options.c_cflag &= ~(CSTOPB);
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            printf("Unsupported stop bits!/n");
            return FALSE;
    }

    if(parity != 'n')
    {
        //修改控制模式，保证程序不会占用串口
        options.c_cflag |=  CLOCAL;
        //修改控制模式，使得能够从串口中读取输入数据    
        options.c_cflag |=  CREAD;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
        options.c_oflag &= ~OPOST;
    }
    //有字符处理或经过TIME个0.1秒后返回       
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    //如果发生数据溢出，接收数据，但是不再读取
    tcflush(fd, TCIFLUSH);
    //激活配置 (将修改后的termios数据设置到串口中）
    if(tcsetattr(fd, TCSANOW, &options) != 0)
    {
        printf("Setup Serial error!\n");
        return FALSE;
    }
    return 0;
}

//写数据函数
int Write_Data(int fd, void *buf, int len)
{
    int m_fd = fd;
    int write_count = 0;
    int nwrite = 0;

    if(m_fd <0)
    {
        printf("Please Open The Device File!\n");
    }
    while(len > 0)
    {
        nwrite = write(fd, (char*)buf + write_count, len);
        if(nwrite < 1)
        {
            printf("Write Datda Fail!\n");
            break;
        }
        write_count += nwrite;
        len -= nwrite;
    }

    // 清除所有正在发生的I/O数据
    tcflush(fd, TCIOFLUSH);
    return write_count;
}

//读取数据
int Read_Data(int fd, char* buff)
{
    int nread = 0;
    int fd_max;
    int nselect;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd,&readfds);
    fd_max = fd+1;

    nselect = select(fd_max, &readfds, NULL, NULL, NULL);
    memset(buff, 0, sizeof(buff));

    if(nselect <= 0)
        printf("select failed");
    else if(FD_ISSET(fd, &readfds) >0)
    {

        nread = read(fd, buff, 8);
        buff[nread] = '\0';
    }

    int j = 0;
    while(buff[j] != '\0')
    {
        printf("the readable data is 0x%x\n", buff[j]);

        j++;
    }

    return nread;
}

//开启模块 
int Open_LaserModule(int fd)
{
    un_char Buff_Open[BUFF_MAXSIZE] = {0x80, 0x06, 0x05, 0x01, 0x74};
    un_char Open_Succeeded[BUFF_MAXSIZE] = {0x80, 0x06, 0x85, 0x01, 0xF4};
    un_char Open_Failed[BUFF_MAXSIZE] = {0x80, 0x06, 0x85, 0x00, 0xF5};

    Write_Data(fd, Buff_Open, strlen(Buff_Open));
    Read_Data(fd, Buff_Open);

    if(strcmp(Buff_Open, Open_Succeeded) == 0)
    {
        printf("Open_LaserModule Succeeded!\n");
    }else if(strcmp(Buff_Open, Open_Failed) == 0)
    {
        printf("Open_LaserModule Failed, Please Try Again!\n");
    }else
    {
        printf("NG\n"); 
        return FALSE;
    }
    return TRUE;
}

//关闭模块 
int Close_LaserModule(int fd)
{
    un_char Buff_Close[BUFF_MAXSIZE] = {0x80, 0x06, 0x05, 0x00, 0x75};
    un_char Close_Succeeded[BUFF_MAXSIZE] = {0x80, 0x06, 0x85, 0x01, 0xF4};
    un_char Close_Failed[BUFF_MAXSIZE] = {0x80, 0x06, 0x85, 0x00, 0xF5};

    //要写的数据中间有0x00，因此把要的数据长度指定为5个字节，而不用strlen()函数计算长度
    Write_Data(fd, Buff_Close, 5);
    Read_Data(fd, Buff_Close);

    if(strcmp(Buff_Close, Close_Succeeded) == 0)
    {
        printf("Close_LaserModule Succeeded!\n");
    }else if(strcmp(Buff_Close, Close_Failed) == 0)
    {
        printf("Close_LaserModule Failed, Please Try Again!\n");
    }else

    {   printf("NG\n"); 
        return FALSE;
    }
    return TRUE;
}

//读取参数
un_char* Read_Parameter(int fd, un_char* device_parameter)
{
    un_char Buff_Read_Parameter[BUFF_MAXSIZE] = {0xFA, 0x06, 0x01, 0xFF};

    Write_Data(fd, Buff_Read_Parameter, strlen(Buff_Read_Parameter));
    Read_Data(fd, Buff_Read_Parameter);
    printf("*******The parameter have read!**********\n");  

    strcpy(device_parameter, Buff_Read_Parameter);
    return device_parameter;
}

//读取机器号,由于串口每次只能读取8个字节，而读取机器号的返回代码有20个字节，所以读取三次数据
un_char* Read_Device_Num(int fd, un_char* device_num)
{
    int nread = 0;

    un_char Buff_Read_DeviceNum[BUFF_MAXSIZE] = {0xFA, 0x06, 0x04, 0xFC};
    un_char Buff_read_back[BUFF_MAXSIZE] = {0};
    un_char *Buff_bp = Buff_read_back;

    Write_Data(fd, Buff_Read_DeviceNum, strlen(Buff_Read_DeviceNum));

    nread = Read_Data(fd, Buff_Read_DeviceNum);
    memcpy(Buff_bp, Buff_Read_DeviceNum, nread);
    Buff_bp += nread;

    nread = Read_Data(fd, Buff_Read_DeviceNum);
    memcpy(Buff_bp, Buff_Read_DeviceNum, nread);
    Buff_bp += nread;

    nread = Read_Data(fd, Buff_Read_DeviceNum);
    strcpy(Buff_bp, Buff_Read_DeviceNum);

    strcpy(device_num, Buff_read_back);
    return device_num;
}

//设置地址
int Set_Address(int fd)
{
    un_char Buff_Address[BUFF_MAXSIZE] = {0xFA, 0x04, 0x01, 0x80, 0x81};
    un_char Address_Succeeded[BUFF_MAXSIZE] = {0xFA, 0x04, 0x81, 0x81};
    un_char Address_Failed[BUFF_MAXSIZE] = {0xFA, 0x84, 0x81, 0x02, 0xFF};

    Write_Data(fd, Buff_Address, strlen(Buff_Address));
    Read_Data(fd, Buff_Address);

    if(strcmp(Buff_Address, Address_Succeeded) == 0)
    {
        printf("Set_Address Succeeded!\n");
    }else if(strcmp(Buff_Address, Address_Failed) == 0)
    {
        printf("Set Address Error, Please Write Again!\n");
    }else

    {       printf("NG\n");
        return FALSE;
    }
    return TRUE;
}

//距离修改,参数decrease_or_increase表示修正可选为取负或者取正，参数distance表示要修正的距离
int Distance_Modification(int fd, int decrease_or_increase, int distance_int)
{
    un_char decrease_increase = (un_char)decrease_or_increase;
    un_char distance_ch = (un_char)distance_int;
    int c_s = -250 - 4 - 6 - decrease_or_increase - distance_int;//十进制250表示十六进制0xFA
    un_char cs = (un_char)c_s;

    un_char Buff_Distance_Mo[BUFF_MAXSIZE] = {0};
    Buff_Distance_Mo[0] = 0xFA;
    Buff_Distance_Mo[1] = 0x04;
    Buff_Distance_Mo[2] = 0x06;
    Buff_Distance_Mo[3] = decrease_increase;
    Buff_Distance_Mo[4] = distance_ch;
    Buff_Distance_Mo[5] = cs;
    Buff_Distance_Mo[6] = '\0';
    un_char Distance_ModificationSd[BUFF_MAXSIZE] ={0xFA, 0x04, 0x8B, 0x77};
    un_char Distance_ModificationFd[BUFF_MAXSIZE] ={0xFA, 0x84, 0x8B, 0x01, 0xF6};

    Write_Data(fd, Buff_Distance_Mo, strlen(Buff_Distance_Mo));
    Read_Data(fd, Buff_Distance_Mo);

    if(strcmp(Buff_Distance_Mo, Distance_ModificationSd) == 0)
    {
        if(decrease_or_increase == 43)
            printf("Distance_Increase_Modification_Succeeded\n");
        else
            printf("Distance_Decrease_Modification_Succeeded\n");
    }else if(strcmp(Buff_Distance_Mo, Distance_ModificationFd) == 0)
    {
        if(decrease_or_increase == 43)
            printf("Distance_Increase_Modification_Failed\n");
        else
            printf("Distance_Decrease_Modification_Failed\n");
    }else
    {
        printf("NG\n");
        return FALSE;
    }
    return TRUE;
}

//连续测量时设置数据返回时间间隔,参数interval_time_int表示要设定的时间间隔为interval_time_int秒
int Mea_Interval(int fd, int interval_time_int)
{
    int c_s = -250 - 4 - 5 - interval_time_int;//十进制250表示十六进制0xFA
    un_char cs = (un_char)c_s;
    un_char interval_ch = (un_char)interval_time_int; 

    un_char Buff_Mea_Interval[BUFF_MAXSIZE] = {0};
    Buff_Mea_Interval[0] = 0xFA;
    Buff_Mea_Interval[1] = 0x04;
    Buff_Mea_Interval[2] = 0x05;
    Buff_Mea_Interval[3] = interval_ch;
    Buff_Mea_Interval[4] = cs;
    Buff_Mea_Interval[5] = '\0';
    un_char Mea_IntervalSd[BUFF_MAXSIZE] = {0xFA, 0x04, 0x85, 0x7D};
    un_char Mea_IntervalFd[BUFF_MAXSIZE] = {0xFA, 0x84, 0x85, 0x01, 0xFC};
    un_char Write_IntervalErr[BUFF_MAXSIZE] = {0xFA, 0x84, 0x85, 0x01, 0xFA};

    Write_Data(fd, Buff_Mea_Interval, strlen(Buff_Mea_Interval));
    Read_Data(fd, Buff_Mea_Interval);

    if(strcmp(Buff_Mea_Interval, Mea_IntervalSd) == 0)
    {
        printf("Mea_Interval_Succeeded\n");
    }else if(strcmp(Buff_Mea_Interval, Mea_IntervalFd) == 0)
    {
        printf("Mea_Interval_Failed, Please Try Again!\n");
    }else if(strcmp(Buff_Mea_Interval, Write_IntervalErr) == 0)
    {
        printf("Mea_Interval_Error\n");
    }else
    {
        printf("NG\n");
        return FALSE;
    }
    return TRUE;
}

//设置距离起止点,参数position_int的值(1顶端算起；0加上模块长度+上面的距离修正)
int Distance_StartStop(int fd, int position_int)
{
    int c_s = -250 - 4 - 8 - position_int;//十进制250表示十六进制0xFA
    un_char cs = (un_char)c_s;
    un_char position_ch = (un_char)position_int;

    un_char Buff_StartStop[BUFF_MAXSIZE] = {0};
    Buff_StartStop[0] = 0xFA;
    Buff_StartStop[1] = 0x04;
    Buff_StartStop[2] = 0x08;
    Buff_StartStop[3] = position_ch;
    Buff_StartStop[4] = cs;
    Buff_StartStop[5] = '\0';
    un_char Mea_Distance_StartStopSd[BUFF_MAXSIZE] = {0xFA, 0x04, 0x88, 0x7A};
    un_char Mea_Distance_StartStopFd[BUFF_MAXSIZE] = {0xFA, 0x84, 0x88, 0x01, 0xF9};

    //要写的数据中间有0x00，因此把要的数据长度指定为5个字节，而不用strlen()函数计算长度
    if(Buff_StartStop[3] == 0x00)
    {
        Write_Data(fd, Buff_StartStop, 5);
    }
    else
    {   
        Write_Data(fd, Buff_StartStop, strlen(Buff_StartStop));
    }
    Read_Data(fd, Buff_StartStop);

    if(strcmp(Buff_StartStop, Mea_Distance_StartStopSd) == 0)
    {
        printf("Mea_Distance_StartStop_Succeeded!\n");
    }else if(strcmp(Buff_StartStop, Mea_Distance_StartStopFd) == 0)
    {
        printf("Write Mea_Distance_StartStop_Failed, Please Write Again!\n");
    }else

    {       printf("NG\n");
        return FALSE;
    }
    return TRUE;
}

//设定量程,range表示要设定的量程大小05, 10, 30, 50, 80
int Set_MeasuringRange(int fd, int range)
{
    un_char Buff_Range_05[BUFF_MAXSIZE] = {0xFA, 0x04, 0x09, 0x05, 0xF4};
    un_char Buff_Range_10[BUFF_MAXSIZE] = {0xFA, 0x04, 0x09, 0x0A, 0xEF};
    un_char Buff_Range_30[BUFF_MAXSIZE] = {0xFA, 0x04, 0x09, 0x1E, 0xDB};
    un_char Buff_Range_50[BUFF_MAXSIZE] = {0xFA, 0x04, 0x09, 0x32, 0xC7};
    un_char Buff_Range_80[BUFF_MAXSIZE] = {0xFA, 0x04, 0x09, 0x50, 0xA9};
    un_char Set_MeasuringRangeSd[BUFF_MAXSIZE] ={0xFA, 0x04, 0x89, 0x79};
    un_char Set_MeasuringRangeFd[BUFF_MAXSIZE] ={0xFA, 0x84, 0x89, 0x01, 0xF8};   

    int choose = range;
    switch(choose)
    {
        case 5:
            Write_Data(fd, Buff_Range_05, strlen(Buff_Range_05));
            Read_Data(fd, Buff_Range_05);

            if(strcmp(Buff_Range_05, Set_MeasuringRangeSd) == 0)
            {
                printf("Set_MeasuringRange_05_Succeeded\n");
            }else if(strcmp(Buff_Range_05, Set_MeasuringRangeFd) == 0)
            {   
                printf("Set_MeasuringRange_05_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n"); 
                return FALSE;
            }
            break;
        case 10:
            Write_Data(fd, Buff_Range_10, strlen(Buff_Range_10));
            Read_Data(fd, Buff_Range_10);

            if(strcmp(Buff_Range_10, Set_MeasuringRangeSd) == 0)
            {
                printf("Set_MeasuringRange_10_Succeeded\n");
            }else if(strcmp(Buff_Range_10, Set_MeasuringRangeFd) == 0)
            {
                printf("Set_MeasuringRange_10_Failed, Please Try Again!\n");
            }else   
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 30:
            Write_Data(fd, Buff_Range_30, strlen(Buff_Range_30));
            Read_Data(fd, Buff_Range_30);

            if(strcmp(Buff_Range_30, Set_MeasuringRangeSd) == 0)
            {
                printf("Set_MeasuringRange_30_Succeeded\n");
            }else if(strcmp(Buff_Range_30, Set_MeasuringRangeFd) == 0)
            {
                printf("Set_MeasuringRange_30_Failed, Please Try Again!\n");
            }else   
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 50:
            Write_Data(fd, Buff_Range_50, strlen(Buff_Range_50));
            Read_Data(fd, Buff_Range_50);

            if(strcmp(Buff_Range_50, Set_MeasuringRangeSd) == 0)
            {
                printf("Set_MeasuringRange_50_Succeeded\n");
            }else if(strcmp(Buff_Range_50, Set_MeasuringRangeFd) == 0)
            {
                printf("Set_MeasuringRange_50_Failed, Please Try Again!\n");
            }else   
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 80:
            Write_Data(fd, Buff_Range_80, strlen(Buff_Range_80));
            Read_Data(fd, Buff_Range_80);

            if(strcmp(Buff_Range_80, Set_MeasuringRangeSd) == 0)
            {
                printf("Set_MeasuringRange_80_Succeeded\n");
            }else if(strcmp(Buff_Range_80, Set_MeasuringRangeFd) == 0)
            {
                printf("Set_MeasuringRange_80_Failed, Please Try Again!\n");
            }else   
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        default:
            printf("Please Set The Correct Range Parameters!\n");
            return FALSE;
    }
    return TRUE;
}

//设定频率,freg表示要设定的频率大小,00,05,10,20
int Set_Frequency(int fd, int freg)
{   
    un_char Buff_Frequency_00[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0A, 0x00, 0xF8};
    un_char Buff_Frequency_05[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0A, 0x05, 0xF3};
    un_char Buff_Frequency_10[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0A, 0x0A, 0xEE};
    un_char Buff_Frequency_20[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0A, 0x14, 0xE4};
    un_char Set_FrequencySd[BUFF_MAXSIZE] ={0xFA, 0x04, 0x8A, 0x78};
    un_char Set_FrequencyFd[BUFF_MAXSIZE] ={0xFA, 0x84, 0x8A, 0x01, 0xF7};
    un_char choose = freg;
    switch(choose)
    {
        case 0:
            Write_Data(fd, Buff_Frequency_00, 5);//要写的数据中间有0x00，因此把要的数据长度指定为5个字节，而不用strlen()函数计算长度
            Read_Data(fd, Buff_Frequency_00);

            if(strcmp(Buff_Frequency_00, Set_FrequencySd) == 0)
            {
                printf("Set_Frequency_00_Succeeded\n");
            }else if(strcmp(Buff_Frequency_00, Set_FrequencyFd) == 0)
            {
                printf("Set_Frequency_00_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n");
                return FALSE;
            }
            break;

        case 5:
            Write_Data(fd, Buff_Frequency_05, strlen(Buff_Frequency_05));
            Read_Data(fd, Buff_Frequency_05);

            if(strcmp(Buff_Frequency_05, Set_FrequencySd) == 0)
            {
                printf("Set_Frequency_05_Succeeded\n");
            }else if(strcmp(Buff_Frequency_05, Set_FrequencyFd) == 0)
            {
                printf("Set_Frequency_05_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 10:
            Write_Data(fd, Buff_Frequency_10, strlen(Buff_Frequency_10));
            Read_Data(fd, Buff_Frequency_10);

            if(strcmp(Buff_Frequency_10, Set_FrequencySd) == 0)
            {
                printf("Set_Frequency_10_Succeeded\n");
            }else if(strcmp(Buff_Frequency_10, Set_FrequencyFd) == 0)
            {
                printf("Set_Frequency_10_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 20:
            Write_Data(fd, Buff_Frequency_20, strlen(Buff_Frequency_20));
            Read_Data(fd, Buff_Frequency_20);

            if(strcmp(Buff_Frequency_20, Set_FrequencySd) == 0)
            {
                printf("Set_Frequency_20_Succeeded\n");
            }else if(strcmp(Buff_Frequency_20, Set_FrequencyFd) == 0)
            {
                printf("Set_Frequency_20_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        default:
            printf("Please Set The Correct Frequency Parameters!\n");
            return FALSE;
    }
    return TRUE;
}

//设定分辨率,当mode=1表示设定的分辨率为1mm,当mode=2表示设定的分辨率为0.1mm;ZPOne为Zero_Point_One
int Set_Resolution(int fd, int mode)
{
    un_char Buff_Resolution_One[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0C, 0x01, 0xF5};
    un_char Buff_Resolution_ZPOne[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0C, 0x02, 0xF4};
    un_char Set_ResolutionSd[BUFF_MAXSIZE] ={0xFA, 0x04, 0x8C, 0x76};
    un_char Set_ResolutionFd[BUFF_MAXSIZE] ={0xFA, 0x84, 0x8C, 0x01, 0xF5};

    int choose = mode;
    //设定分辨率为1mm或者0.1mm
    switch(choose)
    {
        case 1:
            Write_Data(fd, Buff_Resolution_One, strlen(Buff_Resolution_One));
            Read_Data(fd, Buff_Resolution_One);

            if(strcmp(Buff_Resolution_One, Set_ResolutionSd) == 0)
            {
                printf("Set_One_mm_Resolution_Succeeded\n");
            }else if(strcmp(Buff_Resolution_One, Set_ResolutionFd) == 0)
            {
                printf("Set_One_mm_Resolution_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n"); 
                return FALSE;
            }
            break;
        case 2:
            Write_Data(fd, Buff_Resolution_ZPOne, strlen(Buff_Resolution_ZPOne));
            Read_Data(fd, Buff_Resolution_ZPOne);

            if(strcmp(Buff_Resolution_ZPOne, Set_ResolutionSd) == 0)
            {
                printf("Set_ZPOne_mm_Resolution_Succeeded\n");
            }else if(strcmp(Buff_Resolution_ZPOne, Set_ResolutionFd) == 0)
            {
                printf("Set_ZPOne_mm_Resolution_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n"); 
                return FALSE;
            }
            break;
        default:
            printf("Please Set The Correct Resolution Parameters!\n");
            return FALSE;
    }
    return TRUE;
}

//设定上电即测,on_off=1表示开启该功能，on_off=0表示关闭该功能
int Measuring_Power(int fd,int on_off)
{   
    un_char Buff_Measuring_On[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0D, 0x01, 0xF4};
    un_char Buff_Measuring_Off[BUFF_MAXSIZE] = {0xFA, 0x04, 0x0D, 0x00, 0xF5};
    un_char Measuring_PowerSd[BUFF_MAXSIZE] ={0xFA, 0x04, 0x8D, 0x75};
    un_char Measuring_PowerFd[BUFF_MAXSIZE] ={0xFA, 0x84, 0x8D, 0x01, 0xF4};

    int choose = on_off;
    //选择是否开启上电即测
    switch(choose)
    {
        case 0:
            Write_Data(fd, Buff_Measuring_Off, 5);//要写的数据中间有0x00，因此把要的数据长度指定为5个字节，而不用strlen()函数计算长度
            Read_Data(fd, Buff_Measuring_Off);
            if(strcmp(Buff_Measuring_Off, Measuring_PowerSd) == 0)
            {
                printf("Measuring_Power_Off_Succeeded\n");
            }else if(strcmp(Buff_Measuring_Off, Measuring_PowerFd) == 0)
            {
                printf("Measuring_Power_Off_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n");
                return FALSE;
            }
            break;
        case 1:
            Write_Data(fd, Buff_Measuring_On, strlen(Buff_Measuring_On));
            Read_Data(fd, Buff_Measuring_On);

            if(strcmp(Buff_Measuring_On, Measuring_PowerSd) == 0)
            {
                printf("Measuring_Power_On_Succeeded\n");
            }else if(strcmp(Buff_Measuring_On, Measuring_PowerFd) == 0)
            {
                printf("Measuring_Power_On_Failed, Please Try Again!\n");
            }else
            {
                printf("NG\n"); 
                return FALSE;
            }
            break;
        default:
            printf("Please Set The Correct On_Off Mode Parameters!\n");
            return FALSE;           
    }
    return TRUE;
}
//单次测量（广播命令，返回结果存入模块缓存）
int Single_Measurement_Broadcast(int fd)
{
    un_char Buff_Single_MeaBroadcast[BUFF_MAXSIZE] = {0xFA, 0x06, 0x06, 0xFA};

    Write_Data(fd, Buff_Single_MeaBroadcast, strlen(Buff_Single_MeaBroadcast));
    return TRUE;
}

//读取缓存
un_char* Read_Cache(int fd, un_char* cache_data)
{
    int nread = 0;  

    un_char Buff_Read_Cache[BUFF_MAXSIZE] = {0x80, 0x06, 0x07, 0x73};
    un_char Buff_Read_Back[BUFF_MAXSIZE] = {0};
    un_char *Buff_bp = Buff_Read_Back;

    Write_Data(fd, Buff_Read_Cache, strlen(Buff_Read_Cache));

    nread = Read_Data(fd, Buff_Read_Cache);
    memcpy(Buff_bp, Buff_Read_Cache, nread);
    Buff_bp += nread;

    nread = Read_Data(fd, Buff_Read_Cache);
    strcpy(Buff_bp, Buff_Read_Cache);
    //根据返回数据的位数及字符数组的的第四位数据（数组下标为[3]）的值，3X的取值只能是0x30~0x3F(48~63), E(或者e)的大小为69(或者101),判断模块是否正确返回
    if(strlen(Buff_Read_Back)==11 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Read_Cache_on_1mm_Succeeded!\n");
    }else if(strlen(Buff_Read_Back)==12 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Read_Cache_on_0.1mm_Succeeded!\n");
    }else if(Buff_Read_Back[3] == 69 ||Buff_Read_Back[3] == 101)
    {
        if(strlen(Buff_Read_Back)==11)
            printf("Single_Measurement_on_1mm_Failed, Please Try Again!\n");
        if(strlen(Buff_Read_Back)==12)
            printf("Single_Measurement_on_0.1mm_Failed, Please Try Again!\n");
    }else
    {
        printf("NG!\n");
        return NULL;
    }

    strcpy(cache_data, Buff_Read_Back);
    return cache_data;
}

//单次测量
un_char* Single_Measurement(int fd, un_char* single_mea)
{
    int nread = 0;

    un_char Buff_Single_Measurement[BUFF_MAXSIZE] = {0x80, 0x06, 0x02, 0x78};
    un_char Buff_Read_Back[BUFF_MAXSIZE] = {0};
    un_char *Buff_bp = Buff_Read_Back;

    Write_Data(fd, Buff_Single_Measurement, strlen(Buff_Single_Measurement));

    nread = Read_Data(fd, Buff_Single_Measurement);
    memcpy(Buff_bp, Buff_Single_Measurement, nread);
    Buff_bp += nread;

    nread = Read_Data(fd, Buff_Single_Measurement);
    strcpy(Buff_bp, Buff_Single_Measurement);

    //根据返回数据的位数及字符数组的的第四位数据（数组下标为[3]）的值，3X的取值只能是0x30~0x3F(48~63), E(或者e)的大小为69(或者101),判断模块是否正确返回
    if(strlen(Buff_Read_Back)==11 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Single_Measurement_on_1mm_Succeeded!\n");
    }else if(strlen(Buff_Read_Back)==12 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Single_Measurement_on_0.1mm_Succeeded!\n");
    }else if(Buff_Read_Back[3] == 69 ||Buff_Read_Back[3] == 101)
    {
        if(strlen(Buff_Read_Back)==11)
            printf("Single_Measurement_on_1mm_Failed, Please Try Again!\n");
        if(strlen(Buff_Read_Back)==12)
            printf("Single_Measurement_on_0.1mm_Failed, Please Try Again!\n");
    }else
    {
        printf("NG!\n");
        return NULL;
    }

    strcpy(single_mea, Buff_Read_Back);
    return single_mea;  
}

//连续测量
un_char* Continuous_Measurement(int fd, un_char* continuous_mea)
{
    int nread = 0;

    un_char Buff_Continuous_Measurement[BUFF_MAXSIZE] = {0x80, 0x06, 0x03, 0x77};
    un_char Buff_Read_Back[BUFF_MAXSIZE] = {0};
    un_char *Buff_bp = Buff_Read_Back;

    Write_Data(fd, Buff_Continuous_Measurement, strlen(Buff_Continuous_Measurement));

    nread = Read_Data(fd, Buff_Continuous_Measurement);
    memcpy(Buff_bp, Buff_Continuous_Measurement, nread);
    Buff_bp += nread;

    nread = Read_Data(fd, Buff_Continuous_Measurement);
    strcpy(Buff_bp, Buff_Continuous_Measurement);

    //根据返回数据的位数及字符数组的的第四位数据（数组下标为[3]）的值，3X的取值只能是0x30~0x3F(48~63), E(或者e)的大小为69(或者101),判断模块是否正确返回
    if(strlen(Buff_Read_Back)==11 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Continuous_Measurement_on_1mm_Succeeded!\n");
    }else if(strlen(Buff_Read_Back)==12 && Buff_Read_Back[3]>=48 && Buff_Read_Back[3]<=63)
    {
        printf("Continuous_Measurement_on_0.1mm_Succeeded!\n");
    }else if(Buff_Read_Back[3] == 69 ||Buff_Read_Back[3] == 101)
    {
        if(strlen(Buff_Read_Back)==11)
            printf("Continuous_Measurement_on_1mm_Failed, Please Try Again!\n");
        if(strlen(Buff_Read_Back)==12)
            printf("Continuous_Measurement_on_0.1mm_Failed, Please Try Again!\n");
    }else
    {
        printf("NG!\n");
        return NULL;
    }
    strcpy(continuous_mea, Buff_Read_Back);
    return continuous_mea;
}
