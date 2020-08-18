#include "network.h"

/**
 * configure_master_socket: configure the master socket that will facilitate
 * multiple socket connections at the same time.
 */
void configure_master_socket(int *master_socket, int *opt, struct sockaddr_in *address)
{
    //create a master socket  
    if( (*master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)   
    {   
        perror("socket failed");   
        exit(EXIT_FAILURE);   
    }   
     
    //set master socket to allow multiple connections ,  
    //this is just a good habit, it will work without this  
    if ( setsockopt(*master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)opt,  
          sizeof(*opt)) < 0 )   
    {   
        perror("setsockopt");   
        exit(EXIT_FAILURE);   
    }   
     
    //type of socket created  
    address->sin_family = AF_INET;   
    address->sin_addr.s_addr = INADDR_ANY;   
    address->sin_port = htons( PORT );   
         
    //bind the socket to localhost port 8888  
    if (bind(*master_socket, (struct sockaddr *)address, sizeof(*address))<0)   
    {   
        perror("bind failed");   
        exit(EXIT_FAILURE);   
    }   
    printf("Listener on port %d \n", PORT);   
         
    //try to specify maximum of 3 pending connections for the master socket  
    if (listen(*master_socket, 3) < 0)   
    {   
        perror("listen");   
        exit(EXIT_FAILURE);   
    }   
}

/**
 * handle_connection: establish a new connection and attach the new socket
 *  to the master socket
 */
int handle_connection(fd_set *readfds, int *master_socket, 
struct sockaddr_in *address, int addrlen, int client_socket[], int max_clients)
{
    int sd, max_sd, new_socket;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000; // 500ms 
    //clear the socket set  
    FD_ZERO(readfds);   
 
    //add master socket to set  
    FD_SET(*master_socket, readfds);   
    max_sd = *master_socket;   
         
    //add child sockets to set  
    for (int i = 0 ; i < max_clients ; i++)   
    {   
        //socket descriptor  
        sd = client_socket[i];   
             
        //if valid socket descriptor then add to read list  
        if(sd > 0)   
            FD_SET( sd , readfds);   
             
             
        //if valid socket descriptor then add to read list  
        if(sd > 0)   
            FD_SET( sd , readfds);   
             
        //highest file descriptor number, need it for the select function  
        if(sd > max_sd)   
            max_sd = sd;   
    }   
 
    //wait for an activity on one of the sockets , timeout is NULL ,  
    //so wait indefinitely  
    int activity = select( max_sd + 1 , readfds , NULL , NULL , NULL);   
   
    if ((activity < 0) && (errno!=EINTR))   
    {   
        printf("select error");   
    }   
    //If something happened on the master socket ,  
    //then its an incoming connection  
    if (FD_ISSET(*master_socket, readfds))   
    {   
        if ((new_socket = accept(*master_socket,  
                (struct sockaddr *)address, (socklen_t*)&addrlen))<0)   
        {  
            if (errno == EINTR)
            {
                return 1; 
            }
            else
            {
                perror("accept");   
                exit(EXIT_FAILURE);   
            }
        }   
         
        //inform user of socket number - used in send and receive commands  
        /*printf("New connection , socket fd is %d , ip is : %s , port : %d\n", 
            new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   */

        //add new socket to array of sockets  
        for (int j = 0; j < max_clients; j++)   
        {   
            //if position is empty  
            if( client_socket[j] == 0 )   
            {   
                // Set a wait duration at most specified in tv
                setsockopt(new_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
                client_socket[j] = new_socket;   
//                  printf("Adding to list of sockets as %d\n" , i);   
                    
                break;   
            }   
        }   
    }
    return 0;   
}

