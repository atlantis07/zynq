#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

//    serial port set function
void setTermios(struct termios *pNewtio, unsigned short uBaudRate)
{
    bzero(pNewtio,sizeof(struct termios));
    pNewtio->c_cflag = uBaudRate|CS8|CREAD|CLOCAL;
    pNewtio->c_iflag = IGNPAR;
    pNewtio->c_oflag = 0;
    pNewtio->c_lflag = 0;
    pNewtio->c_cc[VINTR] = 0;
    pNewtio->c_cc[VQUIT] = 0;
    pNewtio->c_cc[VERASE] = 0;
    pNewtio->c_cc[VKILL] = 0;
    pNewtio->c_cc[VEOF] = 4;
    pNewtio->c_cc[VTIME] = 5;
    pNewtio->c_cc[VMIN] = 0;
    pNewtio->c_cc[VSWTC] = 0;
    pNewtio->c_cc[VSTART] = 0;
    pNewtio->c_cc[VSTOP] = 0;
    pNewtio->c_cc[VSUSP] = 0;
    pNewtio->c_cc[VEOL] = 0;
    pNewtio->c_cc[VREPRINT] = 0;
    pNewtio->c_cc[VDISCARD] = 0;
    pNewtio->c_cc[VWERASE] = 0;
    pNewtio->c_cc[VLNEXT] = 0;
    pNewtio->c_cc[VEOL2] = 0;
}
int main(int argc,char **argv)
{
    int fd;
    int nCount,nTotal;
    int i,j,m;
    int ReadByte = 0;
    int Buffer[512] = {};
    struct termios oldtio,newtio;
    char *dev = "/dev/ttyGS0";

    if((argc!=3)||(sscanf(argv[1],"%d",&nTotal)!=1))
    {
        printf("Usage:COMSend count datat!\n");
        return -1;
    }

    if((fd=open(dev,O_RDWR|O_NOCTTY|O_NDELAY))<0) //open serial COM2
    {
        printf("Can't open serial port!\n");
        return -1;
    }
    tcgetattr(fd,&oldtio);
    setTermios(&newtio,B9600);
    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

    //Send data
    for(i=0;i<nTotal;i++)
    {
        nCount = write(fd,argv[2],strlen(argv[2]));
        printf("send data OK!count=%d\n",nCount);
        sleep(1);
    }

    //receive data
    for(j=0;;)
    {
        ReadByte = read(fd,Buffer,512);
        sleep(3);
        if(ReadByte>0)
        {
//            printf("readlength=%d\n",ReadByte);
            Buffer[ReadByte]='\0';
            printf("%s\n",Buffer);
            sleep(3);
        }
//        else printf("Read data failure times=%d\n",j);
    }     
    printf("Receive data finished!\n");
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
