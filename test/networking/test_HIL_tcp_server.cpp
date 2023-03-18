#include <chrono>
#include <iostream>
#include <stdint.h>
#include <thread>

#include "chrono_hil/network/tcp/ChTCPClient.h"
#include "chrono_hil/network/tcp/ChTCPServer.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::hil;

int main(int argc, char *argv[]) {
  ChTCPServer server(1209, 3);
  server.Initialize();
  server.Read();
  server.Read();

  return 0;
}