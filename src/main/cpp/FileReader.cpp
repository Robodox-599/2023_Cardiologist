// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FileReader.h"

FileReader::FileReader() = default;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
bool FileReader::ReadConstantsFile(map<string, string> *map)
{
	bool error = false;
	int lineNum = 0;
	string line;
	
  // 1) Open the text file
  	ifstream myfile("/home/lvuser/Constants.txt");

	if (myfile.fail()) { cout << "CONFIG FILE: Cannot open file " << "\n"; return false;}
	
	  // 2) Read each line
    while (getline (myfile,line)) 
    {
			lineNum++;

			// 3) Find the "=" and extract the key and value
			int eqPos = line.find("=");
			if(eqPos < 0) {continue;} // No equals? Not an error, skip the line
					
			string variableName = line.substr(0, eqPos);
			string variableValue = line.substr(eqPos + 1);
					
		  // 4) Make sure the key isn't already in there (flag a problem if so)
			if(map->count(variableName) > 0) { cout << "CONFIG FILE: Duplicate Key [" << variableName << "], Line # " << lineNum << "\n"; error = true; continue; }
		
			// 4a) Check for blank key
			if(variableName.length() == 0) { cout << "CONFIG FILE: Blank Key, line #" << lineNum << endl; error = true; continue; }
					
			// 5) Add the new entry into the map 
			map->insert({variableName, variableValue});
					
    }
  	// 6) Close the file  
    myfile.close();
  
  return !error;
}