#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>
 
#define DEV_NAME  "/dev/ttyUSB2"
 

int main (int argc, char *argv[])
{
	int fd;
	int len, i,ret;
        char buf[] = "ls";
 
 
	fd = open(DEV_NAME, O_RDWR | O_NOCTTY);
        struct  termios Opt;
        tcgetattr(fd, &Opt);
        cfsetispeed(&Opt,B115200); 
        cfsetospeed(&Opt,B115200);
        tcsetattr(fd,TCANOW,&Opt);
        if(fd < 0) {
                perror(DEV_NAME);
                return -1;
        }
 
 
	len = write(fd, buf, sizeof(buf));
	if (len < 0) {
		printf("write data error \n");
	}
	
	len = read(fd, buf, sizeof(buf));
        if (len < 0) {
                printf("read error \n");
                return -1;
        }
 
	printf("%s", buf);
 
	return(0);
}
