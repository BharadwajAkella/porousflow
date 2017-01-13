#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <libconfig.h++>
using namespace libconfig;

Config config;

void initializeConfig(){	

	// Read the file. If there is an error, report it and exit.
	try
	{
		config.readFile("config.cfg");
	}
	catch (const FileIOException &fioex)
	{
		std::cerr << "I/O error while reading file. - " << fioex.what() << std::endl;
		return;
	}
	catch (const ParseException &pex)
	{
		std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
			<< " - " << pex.getError() << std::endl;
		return;
	}
}
#endif