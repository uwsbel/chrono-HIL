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
  std::vector<float> temp = server.GetRecvData();
  for (int i = 0; i < temp.size(); i++)
    std::cout << temp[i] << std::endl;

  sleep(5);

  server.Read();
  std::vector<float> temp2 = server.GetRecvData();
  for (int i = 0; i < temp2.size(); i++)
    std::cout << temp2[i] << std::endl;

  return 0;
}