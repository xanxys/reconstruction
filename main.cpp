#include <iostream>

#include "recon_server.h"

int main() {
	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}
