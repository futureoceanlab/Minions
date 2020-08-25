/**
 * network: a set of functions to help the server setup and handle multiple
 * incoming connections from the clients.
 * 
 * Author: Junsu Jang (junsuj@mit.edu)
 * Date: Aug 25, 2020
 * 
 */
#ifndef NETWORK_H
#define NETWORK_H

#include <stdio.h>  
#include <stdlib.h>  
#include <iostream>
#include <errno.h>  
#include <unistd.h>          
#include <arpa/inet.h>      
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <signal.h>         


#define PORT 8080               // Port number for communication
#define SERVER_IP "192.168.4.1"

/* Configure a master socket that handles multiple sockets */
void configureMasterSocket(int *master_socket, int *opt, struct sockaddr_in *address);
/* Handles incoming connection from the port by clients */
int handleConnection(fd_set *readfds, int *master_socket, struct sockaddr_in *address, int addrlen, int client_socket[], int max_clients);

#endif
