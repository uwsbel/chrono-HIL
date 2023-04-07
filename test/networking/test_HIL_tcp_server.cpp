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

  std::vector<float> write_data;
  write_data.push_back(1.f);
  write_data.push_back(2.f);
  write_data.push_back(3.f);

  server.Write(write_data);

  sleep(5);

  server.Read();
  std::vector<float> temp = server.GetRecvData();
  for (int i = 0; i < temp.size(); i++)
    std::cout << temp[i] << std::endl;

  return 0;
}