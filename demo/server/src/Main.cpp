/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/multiprecision/gmp.hpp>
#include <boost/program_options.hpp>
#include <cassert>
#include <iostream>
#include <vector>

#include "ParseOption.hpp"
#include "ServerCounter.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

using namespace d4;
namespace po = boost::program_options;
MethodManager *methodRun = nullptr;

const unsigned STOP = 0;
const unsigned COUNT = 1;

/**
 * @brief Catch the signal that ask for stopping the method which is running.
 *
 * @param signum is the signal.
 */
static void signalHandler(int signum) {
  std::cout << "c [MAIN] Method stop\n";
  if (methodRun != nullptr) methodRun->interrupt();
  exit(signum);
}  // signalHandler

/**
 * @brief Initialise un socket.
 *
 * @param port est le port utilisé.
 * @return la file descriptor.
 */
int initSocket(int port, int backlog) {
  int fdsocket;
  if ((fdsocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("socket()");
    exit(EXIT_FAILURE);
  }

  int opt = 1;
  if (setsockopt(fdsocket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                 sizeof(opt)) != 0) {
    perror("setsokopt()");
    exit(EXIT_FAILURE);
  }

  struct sockaddr_in adresse;

  adresse.sin_family = AF_INET;
  adresse.sin_addr.s_addr = INADDR_ANY;
  adresse.sin_port = htons(port);

  // bind the socket.
  if (bind(fdsocket, (struct sockaddr *)&adresse, sizeof(adresse)) != 0) {
    perror("bind()");
    exit(EXIT_FAILURE);
  }

  // listen for communication.
  if (listen(fdsocket, backlog) == -1) {
    perror("listen()");
    exit(EXIT_FAILURE);
  }

  return fdsocket;
}  // initSocket

/**
   The main function!
*/
int main(int argc, char **argv) {
  po::options_description desc{"Options"};
  desc.add_options()
#include "option.dsc"
      ;

  signal(SIGINT, signalHandler);
  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try {
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    exit(1);
  }

  // help or problem with the command line
  if (vm.count("help") || !vm.count("port")) {
    if (!vm.count("help"))
      std::cout << "Some parameters are missing, please read the README\n";
    std::cout << "USAGE: " << argv[0] << " -p PORT [OPTIONS]\n";
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }

  int fd = initSocket(vm["port"].as<unsigned>(), 4);

  // wait for the client.
  struct sockaddr_in addrClient;
  socklen_t lenSock = sizeof(struct sockaddr_in);
  int csocket = accept(fd, (struct sockaddr *)&addrClient, &lenSock);
  if (csocket < 0) {
    perror("connect()");
    exit(EXIT_FAILURE);
  }

  std::cout << "[SERVER] Connection established with:"
            << inet_ntoa(addrClient.sin_addr) << ":"
            << ntohs(addrClient.sin_port) << '\n';

  while (true) {
    // get the query.
    int n, code;

    if ((n = recv(csocket, &code, sizeof(code), 0)) < 0) {
      perror("recv()");
      exit(EXIT_FAILURE);
    }

    if (code == STOP) break;
    if (code != COUNT) {
      fprintf(stderr,
              "The query is incorrect, we receive %d and %d is waited\n", code,
              COUNT);
      exit(EXIT_FAILURE);
    }

    // parse the initial problem.
    d4::ProblemManager *initProblem = new ProblemManagerCnf(csocket, true);
    assert(initProblem);
    std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
                 "formula\033[0m\n";
    initProblem->displayStat(std::cout, "c [INITIAL INPUT] ");
    std::cout << "c\n";

    // run the method asked.
    d4::MethodName methodName =
        d4::MethodNameManager::getMethodName("counting");

    // preproc.
    ProblemManager *problem = d4::MethodManager::runPreproc(
        parsePreprocConfiguration(vm), initProblem, std::cout);

    // count.
    serverCounter(vm, problem, csocket);

    delete initProblem;
  }

  std::cout << "[SERVER] Close the connection\n";
  close(csocket);
  close(fd);

  return EXIT_SUCCESS;
}  // main