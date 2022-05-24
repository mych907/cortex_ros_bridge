#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include<iostream>

using namespace std;

int   main( void)
{
    char            host_name[80];
    struct hostent *host_entry;
    int             ndx;

    if ( 0 != gethostname( host_name, sizeof( host_name))){
        printf( "gethostname() 실행 실패\n");
        exit( 1);
    }
    printf( "%s\n", host_name);

    host_entry = gethostbyname( host_name);
    //printf("get host %s\n", host_entry);

    if ( !host_entry){
        printf( "gethostbyname() 실행 실패\n");
        exit( 1);
    }
    for ( ndx = 0; NULL != host_entry->h_addr_list[ndx]; ndx++)
        printf( "%s\n", inet_ntoa( *(struct in_addr*)host_entry->h_addr_list[ndx]));

    return 0;
}