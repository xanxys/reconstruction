#include <iostream>

#include "future_viewer.h"

int main() {
	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}
