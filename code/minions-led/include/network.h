#ifndef NETWORK_H
#define NETWORK_H

#include <stdio.h>  
#include <stdlib.h>  
#include <iostream>
#include <errno.h>  
#include <unistd.h>         //close  
#include <arpa/inet.h>      //close  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <signal.h>         // for 1 second interrupt


#define PORT 8080               // Port number for communication


void configure_master_socket(int *master_socket, int *opt, struct sockaddr_in *address);
int handle_connection(fd_set *readfds, int *master_socket, struct sockaddr_in *address, int addrlen, int client_socket[], int max_clients);

#endif
