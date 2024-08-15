#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

const unsigned int SIZE_BUFFER = 1024;

/**
 * @brief Initialise un socket.
 *
 * @param port est le port utilisé.
 * @return la file descriptor.
 */
int initSocket(int port) {
  int fdsocket;
  if ((fdsocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("socket()");
    exit(EXIT_FAILURE);
  }

  int opt = 1;
  if (setsockopt(fdsocket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                 sizeof(opt)) != 0) {
    perror("setsockopt()");
    exit(EXIT_FAILURE);
  }

  struct sockaddr_in adresse;

  adresse.sin_family = AF_INET;
  adresse.sin_addr.s_addr = INADDR_ANY;
  adresse.sin_port = htons(port);

  if (connect(fdsocket, (struct sockaddr *)&adresse,
              sizeof(struct sockaddr_in)) < 0) {
    perror("connect()");
    exit(errno);
  }

  return fdsocket;
}  // initSocket

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "[USAGE] %s\n");
    return EXIT_FAILURE;
  }

  int port = atoi(argv[1]);

  // initialize the connection with the server.
  int fdsocket = initSocket(port);

  int nbFormula = 2;
  char *formula[] = {"p cnf 3 1\n1 2 0\nz", "p cnf 3 2\n1 2 0 -1 2 0 z"};

  int code;
  for (unsigned i = 0; i < nbFormula; i++) {
    // ask to count
    code = 1;
    if (send(fdsocket, &code, sizeof(code), 0) < 0) {
      perror("send");
      exit(EXIT_FAILURE);
    }

    // send a CNF formula
    char *cnf = formula[i];
    if (send(fdsocket, cnf, strlen(cnf), 0) < 0) {
      perror("sendto()");
      exit(errno);
    }

    // wait for the result.
    int n;
    char buffer[1024];
    if ((n = recv(fdsocket, buffer, sizeof(buffer) - 1, 0)) < 0) {
      perror("recvfrom()");
      exit(errno);
    }
    buffer[n] = '\0';
    printf("s %s\n", buffer, n);
  }

  code = 0;
  if (send(fdsocket, &code, sizeof(code), 0) < 0) {
    perror("send");
    exit(EXIT_FAILURE);
  }

  return EXIT_FAILURE;
}