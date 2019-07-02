#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
int fd;

void *add(void *arg)
	
{	int res;
	while(1)
	{
		
		res=ioctl(fd, 0x89ABCDEF, 0);		//开始休眠，直到有中断发生
		if(res==-1)
		{
			printf("ioctl error\n");
		}
		
/********添加配置代码************************/
											   //来中断，唤醒后执行
		printf("config ...\n");
		system("/etc/init.d/network restart");	//模拟配置
		sleep(10); 							   //模拟等待配置完成 
		printf("config ok\n");				   //配置完成
		
/********添加配置代码************************/		

	    int data = 1;	
		write(fd,&data,1);						//配置完成后写1 使能中断，供下次配置使用
	}
	
}

int main(int argc, char **argv)
{

	
	int res;
	pthread_t tid;
	fd = open("/dev/yq_gpio_dev", O_RDWR);
	if(fd < 0) {
		perror("open error:");
		exit(1);
	}
	printf("open ok\n");
	
	res=pthread_create(&tid,NULL,add,NULL);
	if (res!=0)
	{
		printf("ptherad error\n");
	}

	while(1)
	{
		printf("main thread run...\n");
		sleep(2);
	
	}
		
} 

