/*
    Profile.hpp

    Copyright © 2022 MSAD Mode 2P. All rights reserved.
*/
#ifndef Profile_hpp
#define Profile_hpp

#include <string>
#include <unordered_map>

class Profile {
public:
  Profile(const std::string& path);
  std::string getValueAsStr(const std::string& key);
  double getValueAsNum(const std::string& key);
private:
  std::unordered_map<std::string, std::string> profile;
};

extern Profile*     prof;

#endif /* Profile_hpp */
