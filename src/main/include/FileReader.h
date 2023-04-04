// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <fstream>
#include <iostream>
#include <iomanip> 
#include <map>


using namespace std;

class FileReader {
 public:
  FileReader();
  bool ReadConstantsFile(map<string, string> *map);
 private:
};
