#ifndef _IMAGECLIENT_H_
#define _IMAGECLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class ImageClient
{
  public:
    int Init()
    {
      maxTransSize=1024;//320*240*4;
      hsocket=socket(AF_INET,SOCK_STREAM,0);
      if (hsocket==-1)
      {
        printf("Socket error!\n");
        working=false;
        return 0;
      }
      serv_addr.sin_family=AF_INET;
      serv_addr.sin_port=8888;
      serv_addr.sin_addr.s_addr=inet_addr("134.100.13.191");
      bzero(&(serv_addr.sin_zero),8);
      if (connect(hsocket,(struct sockaddr *)&serv_addr,sizeof(struct sockaddr))==-1)
      {
        printf("Connect error!\n");
        working=false;
        return 0;
      }
      working=true;
      return 1;
    }

    int sendimage(char *buf,int size)
    {
      int i,j;
      if (!working) return 0;
      i=0;
      do
      {
        j=maxTransSize;
        if (j>size-i) j=size-i;
        if (send(hsocket,buf+i,j,0)==-1)
        {
          printf("Send error!\n");
          working=false;
        }
        i+=j;
      } while (i<size);
      return 1;
    }

    void Over()
    {
      if (working) close(hsocket);
    }

  private:
    int maxTransSize;
    int hsocket;
    struct sockaddr_in serv_addr;
    bool working;
};

#endif

