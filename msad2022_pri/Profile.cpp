/*
    Profile.cpp

    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#include "Profile.hpp"
#include "appusr.hpp"

#include <fstream>

Profile::Profile(const std::string& path) {
  std::ifstream is(path);
  std::string key, value;
  std::string delim = "=";
  size_t pos;

  if (!is.is_open()) {
    _log("open failed %s", path.c_str());
  } else {
    for (std::string line; std::getline(is, line);) {
      if ( (pos = line.find(delim)) != std::string::npos ) {
	key = line.substr(0,pos);
	value = line.substr(pos+delim.length(),line.length());
	profile[key] = value;
	_log("%s = %s", key.c_str(), value.c_str());
      }
    }
  }
}

std::string Profile::getValueAsStr(const std::string& key) {
  if ( profile.find(key) == profile.end() ) {
    profile[key] = "";
  }
  return profile[key];
}

double Profile::getValueAsNum(const std::string& key) {
  if ( profile.find(key) == profile.end() ) {
    profile[key] = "0.0";
  }
  return std::stod(profile[key]);
}
